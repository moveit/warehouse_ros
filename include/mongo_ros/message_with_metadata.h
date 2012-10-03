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
 * Defines the MessageWithMetadata class as well as some helper functions
 * to create and manipulate Metadata objects
 *
 * \author Bhaskara Marthi
 */

#ifndef MONGO_ROS_MESSAGE_WITH_METADATA_H
#define MONGO_ROS_MESSAGE_WITH_METADATA_H

#include <mongo_ros/metadata.h>

namespace mongo_ros
{


/************************************************************
 * MessageWithMetadata
 ***********************************************************/


/// \brief Class that wraps (via inheritance) a ROS message type, together
/// with additional metadata (a yaml dictionary)
/// \tparam M the message type being wrapped
template <class M>
struct MessageWithMetadata : public M
{
  MessageWithMetadata (const Metadata& metadata,
                       const M& msg = M()) :
    M(msg), metadata(metadata)
  {
  }

  MessageWithMetadata (const mongo::BSONObj& metadata,
                       const M& msg = M()) :
    M(msg), metadata(metadata)
  {}

  MessageWithMetadata (const MessageWithMetadata& m) :
    M(m), metadata(m.metadata)
  {}

  MessageWithMetadata ()
  {}

  mongo::BSONObj metadata;

  std::string lookupString (const std::string& name) const
  {
    return metadata.getStringField(name.c_str());
  }

  double lookupDouble (const std::string& name) const
  {
    double d;
    metadata[name.c_str()].Val(d);
    return d;
  }

  int lookupInt (const std::string& name) const
  {
    return metadata.getIntField(name.c_str());
  }

  bool lookupBool (const std::string& name) const
  {
    return metadata.getBoolField(name.c_str());
  }

  typedef boost::shared_ptr<MessageWithMetadata<M> > Ptr;
  typedef boost::shared_ptr<MessageWithMetadata<M> const> ConstPtr;
};


} // namespace

#endif // include guard
