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
 * Template implementation for ResultIterator.
 * Only to be included from query_results.h
 *
 * \author Bhaskara Marthi
 */


namespace mongo_ros
{

using std::vector;
using std::string;

template <class M>
ResultIterator<M>::ResultIterator
(boost::shared_ptr<mongo::DBClientConnection> conn,
 const string& ns, const mongo::Query& query,
 boost::shared_ptr<mongo::GridFS> gfs,
 const bool metadata_only) :
  metadata_only_(metadata_only),
  cursor_(new Cursor(conn->query(ns, query))),
  gfs_(gfs)
{
  if ((*cursor_)->more())
    next_ = (*cursor_)->nextSafe();
}

template <class M>
ResultIterator<M>::ResultIterator () :
  metadata_only_(false)
{
}

template <class M>
ResultIterator<M>::ResultIterator (const ResultIterator<M>& res) :
  metadata_only_(res.metadata_only_), cursor_(res.cursor_),
  next_(res.next_), gfs_(res.gfs_)
{
}

template <class M>
void ResultIterator<M>::increment ()
{
  ROS_ASSERT (next_);
  if ((*cursor_)->more())
    next_ = (*cursor_)->nextSafe();
  else
    next_.reset();
}

template <class M>
typename MessageWithMetadata<M>::ConstPtr
ResultIterator<M>::dereference () const
{
  ROS_ASSERT (next_);
  // Get the raw message if necessary, else
  // use a default constructed one
  typename MessageWithMetadata<M>::Ptr m(new MessageWithMetadata<M>(next_->copy()));
  if (!metadata_only_)
  {
    mongo::OID blob_id;
    (*next_)["blob_id"].Val(blob_id);
    mongo::BSONObj q = BSON ("_id" << blob_id);
    mongo::GridFile f = gfs_->findFile(q);
    ROS_ASSERT(f.exists());
    std::stringstream ss (std::ios_base::out);
    f.write(ss);
    std::string str = ss.str();

    uint8_t* buf = (uint8_t*) str.c_str();
    ros::serialization::IStream istream(buf, str.size());
    ros::serialization::Serializer<M>::read(istream, *m);
  }

  return m;
}

template <class M>
bool ResultIterator<M>::equal (const ResultIterator<M>& other) const
{
  // Incomplete; the only case we care about is whether we're at the end yet
  if (next_ && other.next_)
    ROS_WARN ("Unexpected case of equality check of two not-past-the-end "
              "iterators in ResultIterator");
  return (!next_ && !other.next_);
}


} // namespace
