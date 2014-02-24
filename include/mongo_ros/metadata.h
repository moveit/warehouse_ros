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
 * Define a couple of classes that wrap Mongo's BSON type
 *
 * \author Bhaskara Marthi
 */

#ifndef MONGO_ROS_METADATA_H
#define MONGO_ROS_METADATA_H

// We have to include this top-level include here because
// the mongo c++ library is not robust to reincludes
#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif
#include <mongo_ros/config.h>
#include <ros/ros.h>


namespace mongo_ros
{

using mongo::LT;
using mongo::LTE;
using mongo::GT;
using mongo::GTE;
using mongo::BSONObj;
using mongo::BSONObjBuilder;

/// \brief Internal parent class
///
/// This allows the user to not have to deal with separate BSONObj and
/// BSONObj builder objects
class WrappedBSON : public BSONObj
{
public:
  WrappedBSON () :
    BSONObj(), builder_(new BSONObjBuilder())
  {}

  WrappedBSON (const WrappedBSON& other) :
    BSONObj(), builder_(other.builder_)
  {
    update();
  }

  WrappedBSON (const std::string& json) :
    BSONObj(), builder_(new BSONObjBuilder())
  {
    builder_->appendElements(mongo::fromjson(json.c_str()));
    update();
  }

protected:

  boost::shared_ptr<BSONObjBuilder> builder_;

  void update ()
  {
    BSONObj::operator=(builder_->asTempObj());
  }
};


/// \brief Represents a query to the db
///
/// Usage:
/// Query q("foo", 42);
/// Query q2("bar", LT, 24); // bar less than 24
/// Templated so you can have different types of values
///
/// Or:
/// q = Query().append("foo", 42).append("bar", LT, 24);
class Query : public WrappedBSON
{
public:
  Query () : WrappedBSON ()
  {}

  Query (const Query& other) :
    WrappedBSON(other)
  {}

  template <class T>
  Query (const std::string& name, const T& val) :
    WrappedBSON ()
  {
    append(name, val);
  }

  template <class T, class S>
  Query (const std::string& name, const S& rel, const T& val) :
    WrappedBSON ()
  {
    append(name, rel, val);
  }

  template <class T>
  Query& append (const std::string& name,
                 const T& val)
  {
    *builder_ << name << val;
    update();
    return *this;
  }

  template <class T, class S>
  Query& append (const std::string& name,
                       const T& rel,
                       const S& val)
  {
    *builder_ << name << rel << val;
    update();
    return *this;
  }


};



/// \brief Represents metadata attached to a message.  Automatically
/// includes a unique id and creation time.
///
/// Usage:
///
/// Metadata m("x", 24, "y", 42);
/// (templated so you can use varying number of fields, numeric or string values)
/// 
/// Or:
/// m = Metadata().append("x", 24).append("name", "foo");
class Metadata : public WrappedBSON
{
public:
  Metadata() :
    WrappedBSON ()
  {
    initialize();
  }

  Metadata(const std::string& json) :
    WrappedBSON (json)
  {
    update();
  }

  Metadata (const Metadata& other) :
    WrappedBSON(other)
  {
    update();
  }

  template <class T>
  Metadata (const std::string& name, const T& val) :
    WrappedBSON ()
  {
    initialize();
    append(name, val);
  }

  template <class T, class T2>
  Metadata (const std::string& n, const T& v,
            const std::string& n2, const T2& v2) :
    WrappedBSON ()
  {
    initialize();
    append(n, v);
    append(n2, v2);
  }

  template <class T1, class T2, class T3>
  Metadata (const std::string& n1, const T1& v1,
            const std::string& n2, const T2& v2,
            const std::string& n3, const T3& v3) :
    WrappedBSON ()
  {
    initialize();
    append(n1, v1);
    append(n2, v2);
    append(n3, v3);
  }

  template <class T1, class T2, class T3>
  Metadata (const std::string& n1, const T1& v1,
            const std::string& n2, const T2& v2,
            const std::string& n3, const T3& v3,
            const std::string& n4, const T3& v4) :
    WrappedBSON ()
  {
    initialize();
    append(n1, v1);
    append(n2, v2);
    append(n3, v3);
    append(n4, v4);
  }

  template <class T1, class T2, class T3>
  Metadata (const std::string& n1, const T1& v1,
            const std::string& n2, const T2& v2,
            const std::string& n3, const T3& v3,
            const std::string& n4, const T3& v4,
            const std::string& n5, const T3& v5) :
    WrappedBSON ()
  {
    initialize();
    append(n1, v1);
    append(n2, v2);
    append(n3, v3);
    append(n4, v4);
    append(n5, v5);
  }

  template <class T>
  Metadata& append (const std::string& name,
                 const T& val)
  {
    builder_->append(name, val);
    update();
    return *this;
  }

private:

  void initialize()
  {
    builder_->genOID();
    builder_->append("creation_time", ros::Time::now().toSec());
    update();
  }
};




} // namespace

#endif // include guard
