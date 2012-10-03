/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Implementation of mongo_ros.h
 *
 * \author Bhaskara Marthi
 */

#include <mongo_ros/mongo_ros.h>
#include <mongo_ros/exceptions.h>

namespace mongo_ros
{

using std::string;

/// Get a parameter, with default values
template <class P>
P getParam (const ros::NodeHandle& nh, const string& name, const P& default_val) 
{
  P val;
  nh.param(name, val, default_val);
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                          " (default was " << default_val << ")");
  return val;
}

string getHost (ros::NodeHandle nh, const string& host="")
{
  const string db_host =
    (host=="") ?
    getParam<string>(nh, "warehouse_host", "localhost") :
    host;
  return db_host;
}

int getPort (ros::NodeHandle nh, const int port=0)
{
  const int db_port =
    (port==0) ?
    getParam<int>(nh, "warehouse_port", 27017) :
    port;
  return db_port;
}

boost::shared_ptr<mongo::DBClientConnection>
makeDbConnection (const ros::NodeHandle& nh, const string& host,
                  const unsigned& port, const float timeout)
{
  // The defaults should match the ones used by mongodb/wrapper.py
  const string db_host = getHost(nh, host);
  const int db_port = getPort(nh, port);

  const string db_address = (boost::format("%1%:%2%") % db_host % db_port).str();
  boost::shared_ptr<mongo::DBClientConnection> conn;
  
  const ros::WallTime end = ros::WallTime::now() + ros::WallDuration(timeout);
  
  while (ros::ok() && ros::WallTime::now()<end)
  {
    conn.reset(new mongo::DBClientConnection());
    try
    {
      ROS_DEBUG_STREAM_NAMED ("init", "Connecting to db at " << db_address);
      conn->connect(db_address);
      if (!conn->isFailed())
        break;
    }
    catch (mongo::ConnectException& e)
    {
      ros::Duration(1.0).sleep();
    }
  }
  if (conn->isFailed() || ros::WallTime::now()>end)
    throw DbConnectException();

  ROS_DEBUG_STREAM_NAMED("init", "Successfully connected to db");
  return conn;
}

void dropDatabase (const string& db_name)
{
  dropDatabase(db_name, "", 0, 60.0);
}

void dropDatabase (const string& db, const string& host, const unsigned port,
                   const float timeout)
{
  ros::NodeHandle nh;
  boost::shared_ptr<mongo::DBClientConnection> c =
    makeDbConnection(nh, host, port, timeout);
  c->dropDatabase(db);
}

string messageType (mongo::DBClientConnection& conn,
                    const string& db, const string& coll)
{
  const string ns = db+".ros_message_collections";
  std::auto_ptr<mongo::DBClientCursor> cursor = conn.query(ns, BSON("name" << coll));
  mongo::BSONObj obj = cursor->next();
  return obj.getStringField("type");
}

} // namespace
