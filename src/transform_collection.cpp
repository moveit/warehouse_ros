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
 * Implements TransformCollection class
 *
 * \author Bhaskara Marthi
 */

#include <mongo_ros/transform_collection.h>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

namespace mongo_ros
{

namespace gm=geometry_msgs;
using std::string;
using std::vector;
using boost::format;


// We'll look at all transforms between t-history_length and t+INC
const double INC=1.0;
  
mongo::Query TransformCollection::transformQuery (const double t) const
{
  format query("{stamp : { $gte : %.9f, $lte : %.9f } }");
  const double start = t-history_length_;
  const double end = t+INC;
  string s = (query % start % end).str();
  mongo::Query q = mongo::fromjson(s);
  return q;
}

tf::StampedTransform TransformCollection::lookupTransform (const string& target,
                                                           const string& src,
                                                           const double t) const
{
  // Form the query
  const mongo::Query q = transformQuery(t);

  // Iterate over the messages and add them to a Transformer
  tf::Transformer buffer(true, ros::Duration(history_length_+INC*1.1));
  typedef MessageWithMetadata<tf::tfMessage>::ConstPtr MsgPtr;
  BOOST_FOREACH (MsgPtr m, coll_.queryResults(q)) 
  {
    BOOST_FOREACH (const gm::TransformStamped& trans, m->transforms) 
    {
      const gm::Vector3& v = trans.transform.translation;
      const gm::Quaternion& q = trans.transform.rotation;
      const std_msgs::Header& h = trans.header;
      const tf::Transform tr(tf::Quaternion(q.x, q.y, q.z, q.w),
                             tf::Vector3(v.x, v.y, v.z));
      const tf::StampedTransform t(tr, h.stamp, h.frame_id,
                                   trans.child_frame_id);
      const bool ok = buffer.setTransform(t);
      ROS_ASSERT_MSG(ok, "Tf setTransform returned false for transform from %s "
                     "to %s at %.4f", trans.child_frame_id.c_str(),
                     h.frame_id.c_str(), h.stamp.toSec());
    }
  }
  tf::StampedTransform result;
  buffer.lookupTransform(target, src, ros::Time(t), result); // Can throw
  return result;
}


tf::StampedTransform LiveTransformSource::lookupTransform (const string& target,
                                                           const string& src,
                                                           const double tm) const
{
  ros::Time t(tm);
  tf_->waitForTransform(target, src, t, ros::Duration(timeout_));
  tf::StampedTransform trans;
  tf_->lookupTransform(target, src, t, trans); // Can throw
  return trans;
}

} // namespace
