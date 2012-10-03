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
 * Test script for Mongo ros c++ interface
 *
 * \author Bhaskara Marthi
 */

// %Tag(CPP_CLIENT)%

#include "test_mongo_helpers.h"
#include <mongo_ros/message_collection.h>
#include <mongo_ros/exceptions.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>

namespace mr=mongo_ros;
namespace gm=geometry_msgs;
using std::vector;
using std::string;
using std::cout;

typedef mr::MessageWithMetadata<gm::Pose> PoseWithMetadata;
typedef PoseWithMetadata::ConstPtr PoseMetaPtr;

// Helper function that creates metadata for a message.
// Here we'll use the x and y position, as well as a 'name'
// field that isn't part of the original message.
mr::Metadata makeMetadata (const gm::Pose& p, const string& v)
{
  return mr::Metadata("x", p.position.x, "y", p.position.y, "name", v);
}

TEST(MongoRos, MongoRos)
{
  // Symbols used in queries to denote binary predicates for < and >
  // Note that equality is the default, so we don't need a symbol for it
  using mr::LT;
  using mr::GT;

  // Clear existing data if any
  mr::dropDatabase("my_db", "localhost", 27019, 60.0);
  
  // Set up db
  mr::MessageCollection<gm::Pose> coll("my_db", "poses", "localhost",
                                       27019, 60.0);

  // Arrange to index on metadata fields 'x' and 'name'
  coll.ensureIndex("name");
  coll.ensureIndex("x");

  // Add some poses and metadata
  const gm::Pose p1 = makePose(24, 42, 0);
  const gm::Pose p2 = makePose(10, 532, 3);
  const gm::Pose p3 = makePose(53, 22, 5);
  const gm::Pose p4 = makePose(22, -5, 33);
  coll.insert(p1, makeMetadata(p1, "bar"));
  coll.insert(p2, makeMetadata(p2, "baz"));
  coll.insert(p3, makeMetadata(p3, "qux"));
  coll.insert(p1, makeMetadata(p1, "oof"));
  coll.insert(p4, makeMetadata(p4, "ooof"));
  EXPECT_EQ(5u, coll.count());

  // Simple query: find the pose with name 'qux' and return just its metadata
  // Since we're doing an equality check, we don't explicitly specify a predicate
  vector<PoseMetaPtr> res = coll.pullAllResults(mr::Query("name", "qux"), true);
  EXPECT_EQ(1u, res.size());
  EXPECT_EQ("qux", res[0]->lookupString("name"));
  EXPECT_DOUBLE_EQ(53, res[0]->lookupDouble("x"));
  
  // Set up query: position.x < 40 and position.y > 0.  Reverse order
  // by the "name" metadata field.  Also, here we pull the message itself, not
  // just the metadata.  Finally, we can't use the simplified construction
  // syntax here because it's too long
  mr::Query q = mr::Query().append("x", mr::LT, 40).append("y", mr::GT, 0);
  vector<PoseMetaPtr> poses = coll.pullAllResults(q, false, "name", false);
  
  // Verify poses. 
  EXPECT_EQ(3u, poses.size());
  EXPECT_EQ(p1, *poses[0]);
  EXPECT_EQ(p2, *poses[1]);
  EXPECT_EQ(p1, *poses[2]);

  EXPECT_EQ("oof", poses[0]->lookupString("name"));
  EXPECT_EQ("baz", poses[1]->lookupString("name"));
  EXPECT_EQ("bar", poses[2]->lookupString("name"));

  // Set up query to delete some poses.
  mr::Query q2 ("y", mr::LT, 30);

  EXPECT_EQ(5u, coll.count());
  EXPECT_EQ(2u, coll.removeMessages(q2));
  EXPECT_EQ(3u, coll.count());

  // Test findOne
  mr::Query q4("name", "bar");
  EXPECT_EQ(p1, *coll.findOne(q4, false));
  EXPECT_DOUBLE_EQ(24, coll.findOne(q4, true)->lookupDouble("x"));

  mr::Query q5("name", "barbar");
  EXPECT_THROW(coll.findOne(q5, true), mr::NoMatchingMessageException);
  EXPECT_THROW(coll.findOne(q5, false), mr::NoMatchingMessageException);
  
  // Test update
  coll.modifyMetadata(q4, mr::Metadata("name", "barbar"));
  EXPECT_EQ(3u, coll.count());
  EXPECT_THROW(coll.findOne(q4, false), mr::NoMatchingMessageException);
  ROS_INFO("here");
  EXPECT_EQ(p1, *coll.findOne(q5, false));

  // Check stored metadata
  boost::shared_ptr<mongo::DBClientConnection> conn =
    mr::makeDbConnection(ros::NodeHandle());
  EXPECT_EQ("geometry_msgs/Pose", mr::messageType(*conn, "my_db", "poses"));
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "client_test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
 
// %EndTag(CPP_CLIENT)%
