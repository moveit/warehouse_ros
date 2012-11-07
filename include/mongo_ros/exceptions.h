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
 * Exceptions thrown by mongo_ros
 *
 * \author Bhaskara Marthi
 */

#ifndef MONGO_ROS_EXCEPTIONS_H
#define MONGO_ROS_EXCEPTIONS_H

#include <boost/format.hpp>
#include <stdexcept>
#include <string>

namespace mongo_ros
{

using boost::format;
using std::string;

/// A base class for all pose_graph exceptions; provides a handy boost::format parent constructor
class MongoRosException: public std::runtime_error
{
public:
  MongoRosException (const format& error_string) : std::runtime_error(error_string.str()) {};
  MongoRosException (const char* str) : std::runtime_error(str) {};
};

        
/// \brief Couldn't find matching message in collection
struct NoMatchingMessageException: public MongoRosException
{
  NoMatchingMessageException (const string& coll) :
    MongoRosException (format ("Couldn't find message in %1% matching query") % coll) {}
};      

/// \brief Couldn't find matching message in collection
struct DbConnectException: public MongoRosException
{
  DbConnectException () :
    MongoRosException ("Couldn't connect to MongoDB instance") {}
};      

/// \brief Different md5 sum for messages
struct Md5SumException: public MongoRosException
{
  Md5SumException (const string& failure) :
    MongoRosException (format("The md5 sum for the ROS messages saved in the database differs from that of the compiled message. %1%") % failure) {}
};


} // namespace

#endif // include guard
