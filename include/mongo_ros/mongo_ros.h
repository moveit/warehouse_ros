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
 * Db-level operations.  Most operations are in message_collection.h
 *
 * \author Bhaskara Marthi
 */

#ifndef MONGO_ROS_MONGO_ROS_H
#define MONGO_ROS_MONGO_ROS_H

#include <mongo_ros/metadata.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace mongo_ros
{

boost::shared_ptr<mongo::DBClientConnection>
makeDbConnection (const ros::NodeHandle& nh, const std::string& host="",
                  const unsigned& port=0, float timeout=300.0);
  

/// Return the ROS Message type of a given collection
std::string messageType (mongo::DBClientConnection& conn,
                         const std::string& db,
                         const std::string& coll);

/// Drop a db and all its collections
/// Uses the connection parameters given by the warehouse_host and
/// warehouse_port ROS parameters
void dropDatabase (const std::string& db);

/// Drop a db and its collections, given host and port parameters,
/// and a timeout.  A DbClientConnection exception will be thrown if
/// we can't connect in time.
void dropDatabase (const std::string& db, const std::string& host,
                   unsigned port, const float timeout);


} // namespace

#endif // include guard
