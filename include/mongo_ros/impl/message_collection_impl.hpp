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
 * Implementation of template methods of MessageCollection
 * Only to be included by message_collection.h
 *
 * TODO: this should really be in include/impl rather than src/
 *
 * \author Bhaskara Marthi
 */

#include <mongo_ros/exceptions.h>
#include <mongo_ros/mongo_ros.h>
#include <std_msgs/String.h>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

namespace mongo_ros
{

using std::string;
using std::vector;
namespace ser=ros::serialization;
namespace mt=ros::message_traits;


template <class M>
MessageCollection<M>::MessageCollection (const string& db,
                                         const string& coll,
                                         const string& db_host,
                                         const unsigned db_port,
                                         const float timeout) :
  ns_(db+"."+coll),
  md5sum_matches_(true),
  insertion_pub_(nh_.advertise<std_msgs::String>("warehouse/"+db+"/"+coll+\
                                                 "/inserts", 100, true))
{
  initialize(db, coll, db_host, db_port, timeout);
}


template <class M>
void MessageCollection<M>::initialize (const string& db, const string& coll,
                                       const string& host, const unsigned port,
                                       const float timeout)
{
  conn_ = makeDbConnection(nh_, host, port, timeout);
  
  gfs_.reset(new mongo::GridFS(*conn_, db));
  ROS_DEBUG_NAMED ("create_collection", "Constructed collection");

  ensureIndex("creation_time");

  // Add to the metatable
  const string meta_ns = db+".ros_message_collections";
  if (!conn_->count(meta_ns, BSON("name" << coll)))
  {
    ROS_DEBUG_NAMED ("create_collection", "Inserting metadata");
    typedef typename mt::DataType<M> DataType;
    const string datatype = DataType().value();
    typedef typename mt::MD5Sum<M> Md5;
    const string md5 = Md5().value();
    conn_->insert(meta_ns, BSON("name" << coll << "type" << datatype
                                << "md5sum" << md5));
  }
  else
  {
    ROS_DEBUG_NAMED("create_collection", "Not inserting metadata");
    typedef typename mt::MD5Sum<M> Md5;
    const string md5 = Md5().value();
    if (!conn_->count(meta_ns, BSON("name" << coll << "md5sum" << md5)))
    {
      md5sum_matches_ = false;
      typedef typename mt::DataType<M> DataType;
      const string datatype = DataType().value();
      ROS_ERROR("The md5 sum for message %s changed to %s. Only reading metadata.", 
                datatype.c_str(), md5.c_str());
    }
  }
  
  if (insertion_pub_.getNumSubscribers()==0)
  {
    ros::WallDuration d(0.1);
    ROS_DEBUG_STREAM_NAMED ("create_collection",
                            "Waiting " << d.toSec() <<
                            " for any additional notification subscribers");
    d.sleep();
  }
}

template <class M>
MessageCollection<M>& MessageCollection<M>::ensureIndex
(const string& field)
{
  conn_->ensureIndex(ns_, BSON(field << 1));
  return *this;
}

template <class M>
void MessageCollection<M>::insert
(const M& msg, const Metadata& metadata)
{
  if (!md5sum_matches_)
    throw Md5SumException("Cannot insert additional elements.");
  
  /// Get the BSON and id from the metadata
  const mongo::BSONObj bson = metadata;
  mongo::OID id;
  bson["_id"].Val(id);
  
  /// Serialize the message into a buffer
  const size_t serial_size = ser::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ser::OStream stream(buffer.get(), serial_size);
  ser::serialize(stream, msg);
  const char* data = (char*) buffer.get();

  // Store in grid fs
  mongo::BSONObj file_obj = gfs_->storeFile(data, serial_size, id.toString());

  // Add blob id to metadata and store it in the message collection
  mongo::BSONObjBuilder builder;
  builder.appendElements(bson);
  mongo::OID blob_id;
  file_obj["_id"].Val(blob_id);
  builder.append("blob_id", blob_id);
  mongo::BSONObj entry = builder.obj();
  conn_->insert(ns_, entry);
  

  // Publish ROS notification
  std_msgs::String notification;
  notification.data = entry.jsonString();
  insertion_pub_.publish(notification);
}

template <class M>
typename QueryResults<M>::range_t
MessageCollection<M>::queryResults (const mongo::Query& query,
                                    const bool metadata_only,
                                    const string& sort_by,
                                    const bool ascending) const
{
  if (!md5sum_matches_ && !metadata_only)
    throw Md5SumException("Can only query metadata.");
  
  mongo::Query copy(query.obj);
  ROS_DEBUG_NAMED("query", "Sending query %s to %s", copy.toString().c_str(),
                  ns_.c_str());
                  
  if (sort_by.size() > 0)
    copy.sort(sort_by, ascending ? 1 : -1);
  return typename QueryResults<M>::range_t
    (ResultIterator<M>(conn_, ns_, copy, gfs_, metadata_only),
     ResultIterator<M>());
}


template <class M>
vector <typename MessageWithMetadata<M>::ConstPtr>
MessageCollection<M>::pullAllResults (const mongo::Query& query,
                                      const bool metadata_only,
                                      const string& sort_by,
                                      const bool ascending) const
{  
  typename QueryResults<M>::range_t res = queryResults(query, metadata_only,
                                                       sort_by, ascending);
  return vector<typename MessageWithMetadata<M>::ConstPtr>
    (res.first, res.second);
}


template <class M>
typename MessageWithMetadata<M>::ConstPtr
MessageCollection<M>::findOne (const Query& q,const bool metadata_only) const
{
  typename QueryResults<M>::range_t res = queryResults(q, metadata_only);
  if (res.first==res.second)
    throw NoMatchingMessageException(ns_);
  return *res.first;
}


template <class M>
unsigned MessageCollection<M>::removeMessages (const mongo::Query& query)
{
  unsigned num_removed = 0;
  typedef typename MessageWithMetadata<M>::ConstPtr Msg;
  vector<Msg> msgs = pullAllResults(query, true);
  conn_->remove(ns_, query);

  // Also remove the raw messages from gridfs
  BOOST_FOREACH (Msg m, msgs)
  {
    mongo::OID id;
    m->metadata["_id"].Val(id);
    gfs_->removeFile(id.toString());
    num_removed++;
  }

  return num_removed;
}

template <class M>
void MessageCollection<M>::modifyMetadata (const Query& q, const Metadata& m)
{
  typename MessageWithMetadata<M>::ConstPtr orig = findOne(q, true);
  
  mongo::BSONObjBuilder new_meta_builder;

  std::set<std::string> fields;
  m.getFieldNames(fields);

  BOOST_FOREACH (const string& f, fields) 
  {
    if ((f!="_id") && (f!="creation_time")) 
      new_meta_builder.append(BSON("$set" << BSON(f << m.getField(f))).\
                              getField("$set"));
  }

  mongo::BSONObj new_meta = new_meta_builder.obj().copy();
  conn_->update(ns_, q, new_meta);
}

template <class M>
unsigned MessageCollection<M>::count ()
{
  return conn_->count(ns_);
}

template <class M>
bool MessageCollection<M>::md5SumMatches () const
{
  return md5sum_matches_;
}

} // namespace
