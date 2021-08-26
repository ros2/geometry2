/*
 * Copyright (c) 2010, Willow Garage, Inc.
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
 */

/** \author Tully Foote */

#include "tf2/buffer_core.h"
#include "tf2/time_cache.h"
#include "tf2/exceptions.h"

#include <assert.h>
#include <console_bridge/console.h>
#include "tf2/LinearMath/Transform.h"

namespace tf2
{

// Tolerance for acceptable quaternion normalization
static double QUATERNION_NORMALIZATION_TOLERANCE = 10e-3;

/** \brief convert Transform msg to Transform */
void transformMsgToTF2(const geometry_msgs::msg::Transform& msg, tf2::Transform& tf2)
{tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::Transform& msg)
{
  msg.translation.x = tf2.getOrigin().x();
  msg.translation.y = tf2.getOrigin().y();
  msg.translation.z = tf2.getOrigin().z();
  msg.rotation.x = tf2.getRotation().x();
  msg.rotation.y = tf2.getRotation().y();
  msg.rotation.z = tf2.getRotation().z();
  msg.rotation.w = tf2.getRotation().w();
}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::TransformStamped& msg, builtin_interfaces::msg::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(tf2, msg.transform);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos, geometry_msgs::msg::Transform& msg)
{
  msg.translation.x = pos.x();
  msg.translation.y = pos.y();
  msg.translation.z = pos.z();
  msg.rotation.x = orient.x();
  msg.rotation.y = orient.y();
  msg.rotation.z = orient.z();
  msg.rotation.w = orient.w();
}

void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos, geometry_msgs::msg::TransformStamped& msg, builtin_interfaces::msg::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(orient, pos, msg.transform);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

void setIdentity(geometry_msgs::msg::Transform& tx)
{
  tx.translation.x = 0;
  tx.translation.y = 0;
  tx.translation.z = 0;
  tx.rotation.x = 0;
  tx.rotation.y = 0;
  tx.rotation.z = 0;
  tx.rotation.w = 1;
}

bool startsWithSlash(const std::string& frame_id)
{
  if (frame_id.size() > 0)
    if (frame_id[0] == '/')
      return true;
  return false;
}

std::string stripSlash(const std::string& in)
{
  std::string out = in;
  if (startsWithSlash(in))
    out.erase(0,1);
  return out;
}

namespace {

void fillOrWarnMessageForInvalidFrame(
  const char* function_name_arg,
  const std::string& frame_id,
  std::string* error_msg,
  const char* rationale)
{
  std::string s = "Invalid frame ID \"" + frame_id +
                  "\" passed to " + function_name_arg + " - " + rationale;
  if (error_msg != nullptr)
  {
    *error_msg = s;
  }
  else
  {
    CONSOLE_BRIDGE_logWarn("%s", s.c_str());
  }
}

}  // anonymous namespace

CompactFrameID BufferCore::validateFrameId(
  const char* function_name_arg,
  const std::string& frame_id,
  std::string* error_msg) const
{
  if (frame_id.empty())
  {
    fillOrWarnMessageForInvalidFrame(
      function_name_arg, frame_id, error_msg, "in tf2 frame_ids cannot be empty");
    return 0;
  }

  if (startsWithSlash(frame_id))
  {
    fillOrWarnMessageForInvalidFrame(
      function_name_arg, frame_id, error_msg, "in tf2 frame_ids cannot start with a '/'");
    return 0;
  }

  CompactFrameID id = lookupFrameNumber(frame_id);
  if (id == 0) {
    fillOrWarnMessageForInvalidFrame(
      function_name_arg, frame_id, error_msg, "frame does not exist");
  }
  return id;
}

CompactFrameID BufferCore::validateFrameId(const char* function_name_arg, const std::string& frame_id) const
{
  if (frame_id.empty())
  {
    std::string error_msg = "Invalid argument \"" + frame_id + "\" passed to " + function_name_arg + " - in tf2 frame_ids cannot be empty";
    throw tf2::InvalidArgumentException(error_msg.c_str());
  }

  if (startsWithSlash(frame_id))
  {
    std::string error_msg = "Invalid argument \"" + frame_id + "\" passed to " + function_name_arg + " - in tf2 frame_ids cannot start with a '/'";
    throw tf2::InvalidArgumentException(error_msg.c_str());
  }

  CompactFrameID id = lookupFrameNumber(frame_id);
  if (id == 0)
  {
    std::string error_msg = "\"" + frame_id + "\" passed to " + function_name_arg + " does not exist. ";
    throw tf2::LookupException(error_msg.c_str());
  }
  
  return id;
}

BufferCore::BufferCore(tf2::Duration cache_time)
: cache_time_(cache_time)
, transformable_callbacks_counter_(0)
, transformable_requests_counter_(0)
, using_dedicated_thread_(false)
{
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(TimeCacheInterfacePtr());
  frameIDs_reverse.push_back("NO_PARENT");
}

BufferCore::~BufferCore()
{

}

void BufferCore::clear()
{
  //old_tf_.clear();


  std::unique_lock<std::mutex> lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    for (std::vector<TimeCacheInterfacePtr>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      if (*cache_it)
        (*cache_it)->clearList();
    }
  }
  
}

bool BufferCore::setTransform(const geometry_msgs::msg::TransformStamped& transform, const std::string & authority, bool is_static)
{
  tf2::Transform tf2_transform(tf2::Quaternion(transform.transform.rotation.x,
                                               transform.transform.rotation.y,
                                               transform.transform.rotation.z,
                                               transform.transform.rotation.w),
                               tf2::Vector3(transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z));
  TimePoint time_point(std::chrono::nanoseconds(transform.header.stamp.nanosec) +
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(transform.header.stamp.sec)));
  return setTransformImpl(tf2_transform, transform.header.frame_id, transform.child_frame_id,
                          time_point, authority, is_static);
}

bool BufferCore::setTransformImpl(const tf2::Transform& transform_in, const std::string frame_id,
                                  const std::string child_frame_id, const TimePoint stamp,
                                  const std::string& authority, bool is_static)
{

  /////BACKEARDS COMPATABILITY 
  /* tf::StampedTransform tf_transform;
  tf::transformStampedMsgToTF(transform_in, tf_transform);
  if  (!old_tf_.setTransform(tf_transform, authority))
  {
    printf("Warning old setTransform Failed but was not caught\n");
    }*/

  /////// New implementation
  std::string stripped_frame_id = stripSlash(frame_id);
  std::string stripped_child_frame_id = stripSlash(child_frame_id);


  bool error_exists = false;
  if (stripped_child_frame_id == stripped_frame_id)
  {
    CONSOLE_BRIDGE_logError("TF SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), stripped_child_frame_id.c_str());
    error_exists = true;
  }

  if (stripped_child_frame_id == "")
  {
    CONSOLE_BRIDGE_logError("TF NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
    error_exists = true;
  }

  if (stripped_frame_id == "")
  {
    CONSOLE_BRIDGE_logError("TF NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", stripped_child_frame_id.c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(transform_in.getOrigin().x()) || std::isnan(transform_in.getOrigin().y()) || std::isnan(transform_in.getOrigin().z())||
      std::isnan(transform_in.getRotation().x()) || std::isnan(transform_in.getRotation().y()) || std::isnan(transform_in.getRotation().z()) || std::isnan(transform_in.getRotation().w()))
  {
    CONSOLE_BRIDGE_logError("TF NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
             stripped_child_frame_id.c_str(), authority.c_str(),
             transform_in.getOrigin().x(), transform_in.getOrigin().y(), transform_in.getOrigin().z(),
             transform_in.getRotation().x(), transform_in.getRotation().y(), transform_in.getRotation().z(), transform_in.getRotation().w()
              );
    error_exists = true;
  }

  bool valid = std::abs((transform_in.getRotation().w() * transform_in.getRotation().w()
                         + transform_in.getRotation().x() * transform_in.getRotation().x()
                         + transform_in.getRotation().y() * transform_in.getRotation().y()
                         + transform_in.getRotation().z() * transform_in.getRotation().z()) - 1.0f) < QUATERNION_NORMALIZATION_TOLERANCE;

  if (!valid) 
  {
    CONSOLE_BRIDGE_logError("TF DENORMALIZED_QUATERNION: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of an invalid quaternion in the transform (%f %f %f %f)",
             stripped_child_frame_id.c_str(), authority.c_str(),
             transform_in.getRotation().x(), transform_in.getRotation().y(), transform_in.getRotation().z(), transform_in.getRotation().w());
    error_exists = true;
  }

  if (error_exists)
    return false;
  
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    CompactFrameID frame_number = lookupOrInsertFrameNumber(stripped_child_frame_id);
    TimeCacheInterfacePtr frame = getFrame(frame_number);
    if (frame == NULL)
    {
      frame = allocateFrame(frame_number, is_static);
    }
    else 
    {
      // Overwrite TimeCacheInterface type with a current input
      const TimeCache* time_cache_ptr = dynamic_cast<TimeCache*>(frame.get());
      const StaticCache* static_cache_ptr = dynamic_cast<StaticCache*>(frame.get());
      if (time_cache_ptr && is_static)
      {
        frame = allocateFrame(frame_number, is_static);
      }
      else if (static_cache_ptr && !is_static)
      {
        frame = allocateFrame(frame_number, is_static);
      }
    }

    if (frame->insertData(TransformStorage(stamp, transform_in.getRotation(), transform_in.getOrigin(), lookupOrInsertFrameNumber(stripped_frame_id), frame_number)))
    {
      frame_authority_[frame_number] = authority;
    }
    else
    {
      std::string stamp_str = displayTimePoint(stamp);
      CONSOLE_BRIDGE_logWarn("TF_OLD_DATA ignoring data from the past for frame %s at time %s according to authority %s\nPossible reasons are listed at http://wiki.ros.org/tf/Errors%%20explained", stripped_child_frame_id.c_str(), stamp_str.c_str(), authority.c_str());
      return false;
    }
  }

  testTransformableRequests();

  return true;
}

TimeCacheInterfacePtr BufferCore::allocateFrame(CompactFrameID cfid, bool is_static)
{
  TimeCacheInterfacePtr frame_ptr = frames_[cfid];
  if (is_static) {
    frames_[cfid] = TimeCacheInterfacePtr(new StaticCache());
  } else {
    frames_[cfid] = TimeCacheInterfacePtr(new TimeCache(cache_time_));
  }

  return frames_[cfid];
}

enum WalkEnding
{
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

// TODO for Jade: Merge walkToTopParent functions; this is now a stub to preserve ABI
template<typename F>
tf2::TF2Error BufferCore::walkToTopParent(F& f, TimePoint time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const
{
  return walkToTopParent(f, time, target_id, source_id, error_string, NULL);
}

template<typename F>
tf2::TF2Error BufferCore::walkToTopParent(F& f, TimePoint time, CompactFrameID target_id,
    CompactFrameID source_id, std::string* error_string, std::vector<CompactFrameID>
    *frame_chain) const
{
  if (frame_chain)
    frame_chain->clear();

  // Short circuit if zero length transform to allow lookups on non existant links
  if (source_id == target_id)
  {
    f.finalize(Identity, time);
    return tf2::TF2Error::NO_ERROR;
  }

  //If getting the latest get the latest common time
  if (time == TimePointZero)
  {
    tf2::TF2Error retval = getLatestCommonTime(target_id, source_id, time, error_string);
    if (retval != tf2::TF2Error::NO_ERROR)
    {
      return retval;
    }
  }

  // Walk the tree to its root from the source frame, accumulating the transform
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  uint32_t depth = 0;

  std::string extrapolation_error_string;
  bool extrapolation_might_have_occurred = false;

  while (frame != 0)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);
    if (frame_chain)
      frame_chain->push_back(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, &extrapolation_error_string);
    if (parent == 0)
    {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      extrapolation_might_have_occurred = true;
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      f.finalize(TargetParentOfSource, time);
      return tf2::TF2Error::NO_ERROR;
    }

    f.accum(true);

    top_parent = frame;
    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return tf2::TF2Error::LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its transform
  frame = target_id;
  depth = 0;
  std::vector<CompactFrameID> reverse_frame_chain;

  while (frame != top_parent)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);
    if (frame_chain)
      reverse_frame_chain.push_back(frame);

    if (!cache)
    {
      break;
    }

    CompactFrameID parent = f.gather(cache, time, error_string);
    if (parent == 0)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      return tf2::TF2Error::EXTRAPOLATION_ERROR;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      f.finalize(SourceParentOfTarget, time);
      if (frame_chain)
      {
        frame_chain->swap(reverse_frame_chain);
      }
      return tf2::TF2Error::NO_ERROR;
    }

    f.accum(false);

    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return tf2::TF2Error::LOOKUP_ERROR;
    }
  }

  if (frame != top_parent)
  {
    if (extrapolation_might_have_occurred)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << extrapolation_error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      return tf2::TF2Error::EXTRAPOLATION_ERROR;

    }

    createConnectivityErrorString(source_id, target_id, error_string);
    return tf2::TF2Error::CONNECTIVITY_ERROR;
  }

  f.finalize(FullPath, time);
  if (frame_chain)
  {
    // Pruning: Compare the chains starting at the parent (end) until they differ
    int m = (int)reverse_frame_chain.size()-1;
    int n = (int)frame_chain->size()-1;
    for (; m >= 0 && n >= 0; --m, --n)
    {
      if ((*frame_chain)[n] != reverse_frame_chain[m])
        break;
    }
    // Erase all duplicate items from frame_chain
    if (n > 0)
      frame_chain->erase(frame_chain->begin() + (n-1), frame_chain->end());

    if (m < reverse_frame_chain.size())
    {
      for (int i = m; i >= 0; --i)
      {
        frame_chain->push_back(reverse_frame_chain[i]);
      }
    }
  }

  return tf2::TF2Error::NO_ERROR;
}



struct TransformAccum
{
  TransformAccum()
  : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(0.0, 0.0, 0.0, 1.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  CompactFrameID gather(TimeCacheInterfacePtr cache, TimePoint time, std::string* error_string)
  {
    if (!cache->getData(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
      source_to_top_quat = st.rotation_ * source_to_top_quat;
    }
    else
    {
      target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
      target_to_top_quat = st.rotation_ * target_to_top_quat;
    }
  }

  void finalize(WalkEnding end, TimePoint _time)
  {
    switch (end)
    {
    case Identity:
      break;
    case TargetParentOfSource:
      result_vec = source_to_top_vec;
      result_quat = source_to_top_quat;
      break;
    case SourceParentOfTarget:
      {
        tf2::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf2::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        tf2::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf2::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

     	result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      }
      break;
    };

    time = _time;
  }

  TransformStorage st;
  TimePoint time;
  tf2::Quaternion source_to_top_quat;
  tf2::Vector3 source_to_top_vec;
  tf2::Quaternion target_to_top_quat;
  tf2::Vector3 target_to_top_vec;

  tf2::Quaternion result_quat;
  tf2::Vector3 result_vec;
};

geometry_msgs::msg::TransformStamped 
  BufferCore::lookupTransform(const std::string& target_frame, const std::string& source_frame,
      const TimePoint& time) const
{
  tf2::Transform transform;
  TimePoint time_out;
  lookupTransformImpl(target_frame, source_frame, time, transform, time_out);
  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = transform.getOrigin().x();
  msg.transform.translation.y = transform.getOrigin().y();
  msg.transform.translation.z = transform.getOrigin().z();
  msg.transform.rotation.x = transform.getRotation().x();
  msg.transform.rotation.y = transform.getRotation().y();
  msg.transform.rotation.z = transform.getRotation().z();
  msg.transform.rotation.w = transform.getRotation().w();
  std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time_out.time_since_epoch());
  std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(time_out.time_since_epoch());
  msg.header.stamp.sec = (int32_t)s.count();
  msg.header.stamp.nanosec = (uint32_t)(ns.count() % 1000000000ull);
  msg.header.frame_id = target_frame;
  msg.child_frame_id = source_frame;

  return msg;
}

geometry_msgs::msg::TransformStamped
  BufferCore::lookupTransform(const std::string& target_frame, const TimePoint& target_time,
      const std::string& source_frame, const TimePoint& source_time,
      const std::string& fixed_frame) const
{
  tf2::Transform transform;
  TimePoint time_out;
  lookupTransformImpl(target_frame, target_time, source_frame, source_time,
                      fixed_frame, transform, time_out);
  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = transform.getOrigin().x();
  msg.transform.translation.y = transform.getOrigin().y();
  msg.transform.translation.z = transform.getOrigin().z();
  msg.transform.rotation.x = transform.getRotation().x();
  msg.transform.rotation.y = transform.getRotation().y();
  msg.transform.rotation.z = transform.getRotation().z();
  msg.transform.rotation.w = transform.getRotation().w();
  std::chrono::nanoseconds ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time_out.time_since_epoch());
  std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(time_out.time_since_epoch());
  msg.header.stamp.sec = (int32_t)s.count();
  msg.header.stamp.nanosec = (uint32_t)(ns.count() % 1000000000ull);
  msg.header.frame_id = target_frame;
  msg.child_frame_id = source_frame;

  return msg;
}


void BufferCore::lookupTransformImpl(const std::string& target_frame,
                                                            const std::string& source_frame,
                                                            const TimePoint& time, tf2::Transform& transform,
                                                            TimePoint& time_out) const
{
  std::unique_lock<std::mutex> lock(frame_mutex_);

  if (target_frame == source_frame) {
    transform.setIdentity();

    if (time == TimePointZero)
    {
      CompactFrameID target_id = lookupFrameNumber(target_frame);
      TimeCacheInterfacePtr cache = getFrame(target_id);
      if (cache)
        time_out = cache->getLatestTimestamp();
      else
        time_out = time;
    }
    else
      time_out = time;
    return;
  }

  // Identity case does not need to be validated
  CompactFrameID target_id = validateFrameId("lookupTransform argument target_frame", target_frame);
  CompactFrameID source_id = validateFrameId("lookupTransform argument source_frame", source_frame);

  std::string error_string;
  TransformAccum accum;
  tf2::TF2Error retval = walkToTopParent(accum, time, target_id, source_id, &error_string);
  if (retval != tf2::TF2Error::NO_ERROR)
  {
    switch (retval)
    {
    case tf2::TF2Error::CONNECTIVITY_ERROR:
      throw ConnectivityException(error_string);
    case tf2::TF2Error::EXTRAPOLATION_ERROR:
      throw ExtrapolationException(error_string);
    case tf2::TF2Error::LOOKUP_ERROR:
      throw LookupException(error_string);
    default:
      CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
      assert(0);
    }
  }

  time_out = accum.time;
  transform.setOrigin(accum.result_vec);
  transform.setRotation(accum.result_quat);
}

                                                       
void BufferCore::lookupTransformImpl(const std::string& target_frame,
                                                        const TimePoint& target_time,
                                                        const std::string& source_frame,
                                                        const TimePoint& source_time,
                                                        const std::string& fixed_frame, tf2::Transform& transform,
                                                        TimePoint& time_out) const
{
  validateFrameId("lookupTransform argument target_frame", target_frame);
  validateFrameId("lookupTransform argument source_frame", source_frame);
  validateFrameId("lookupTransform argument fixed_frame", fixed_frame);

  tf2::Transform tf1, tf2;

  lookupTransformImpl(fixed_frame, source_frame, source_time, tf1, time_out);
  lookupTransformImpl(target_frame, fixed_frame, target_time, tf2, time_out);

  transform = tf2*tf1;
}



/*
geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const tf2::TimePoint& time, 
                                          const tf2::Duration& averaging_interval) const
{
  try
  {
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, 
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}

geometry_msgs::Twist BufferCore::lookupTwist(const std::string& tracking_frame, 
                                          const std::string& observation_frame, 
                                          const std::string& reference_frame,
                                          const tf2::Point & reference_point, 
                                          const std::string& reference_point_frame, 
                                          const tf2::TimePoint& time, 
                                          const tf2::Duration& averaging_interval) const
{
  try{
  geometry_msgs::Twist t;
  old_tf_.lookupTwist(tracking_frame, observation_frame, reference_frame, reference_point, reference_point_frame,
                      time, averaging_interval, t);
  return t;
  }
  catch (tf::LookupException& ex)
  {
    throw tf2::LookupException(ex.what());
  }
  catch (tf::ConnectivityException& ex)
  {
    throw tf2::ConnectivityException(ex.what());
  }
  catch (tf::ExtrapolationException& ex)
  {
    throw tf2::ExtrapolationException(ex.what());
  }
  catch (tf::InvalidArgument& ex)
  {
    throw tf2::InvalidArgumentException(ex.what());
  }
}
*/

struct CanTransformAccum
{
  CompactFrameID gather(TimeCacheInterfacePtr cache, TimePoint time, std::string* error_string)
  {
    return cache->getParent(time, error_string);
  }

  void accum(bool source)
  {
  }

  void finalize(WalkEnding end, TimePoint _time)
  {
  }

  TransformStorage st;
};

bool BufferCore::canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
                    const TimePoint& time, std::string* error_msg) const
{
  if (target_id == 0 || source_id == 0)
  {
    if (error_msg)
    {
      *error_msg = "Source or target frame is not yet defined";
    }
    return false;
  }

  if (target_id == source_id)
  {
    return true;
  }

  CanTransformAccum accum;
  if (walkToTopParent(accum, time, target_id, source_id, error_msg) == tf2::TF2Error::NO_ERROR)
  {
    return true;
  }

  return false;
}

bool BufferCore::canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
                                  const TimePoint& time, std::string* error_msg) const
{
  std::unique_lock<std::mutex> lock(frame_mutex_);
  return canTransformNoLock(target_id, source_id, time, error_msg);
}

bool BufferCore::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const TimePoint& time, std::string* error_msg) const
{
  // Short circuit if target_frame == source_frame
  if (target_frame == source_frame)
    return true;

  CompactFrameID target_id = validateFrameId(
    "canTransform argument target_frame", target_frame, error_msg);
  if (target_id == 0)
  {
    return false;
  }
  CompactFrameID source_id = validateFrameId(
    "canTransform argument source_frame", source_frame, error_msg);
  if (source_id == 0)
  {
    return false;
  }

  return canTransformInternal(target_id, source_id, time, error_msg);
}

bool BufferCore::canTransform(const std::string& target_frame, const TimePoint& target_time,
                          const std::string& source_frame, const TimePoint& source_time,
                          const std::string& fixed_frame, std::string* error_msg) const
{
  CompactFrameID target_id = validateFrameId(
    "canTransform argument target_frame", target_frame, error_msg);
  if (target_id == 0)
  {
    return false;
  }
  CompactFrameID source_id = validateFrameId(
    "canTransform argument source_frame", source_frame, error_msg);
  if (source_id == 0)
  {
    return false;
  }
  CompactFrameID fixed_id = validateFrameId(
    "canTransform argument fixed_frame", fixed_frame, error_msg);
  if (fixed_id == 0)
  {
    return false;
  }
  
  return
    canTransformInternal(target_id, fixed_id, target_time, error_msg) &&
    canTransformInternal(fixed_id, source_id, source_time, error_msg);
}


tf2::TimeCacheInterfacePtr BufferCore::getFrame(CompactFrameID frame_id) const
{
  if (frame_id >= frames_.size())
    return TimeCacheInterfacePtr();
  else
  {
    return frames_[frame_id];
  }
}

CompactFrameID BufferCore::lookupFrameNumber(const std::string& frameid_str) const
{
  CompactFrameID retval;
  M_StringToCompactFrameID::const_iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(0);
  }
  else
    retval = map_it->second;
  return retval;
}

CompactFrameID BufferCore::lookupOrInsertFrameNumber(const std::string& frameid_str)
{
  CompactFrameID retval = 0;
  M_StringToCompactFrameID::iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(frames_.size());
    frames_.push_back(TimeCacheInterfacePtr());//Just a place holder for iteration
    frameIDs_[frameid_str] = retval;
    frameIDs_reverse.push_back(frameid_str);
  }
  else
    retval = frameIDs_[frameid_str];

  return retval;
}

const std::string& BufferCore::lookupFrameString(CompactFrameID frame_id_num) const
{
    if (frame_id_num >= frameIDs_reverse.size())
    {
      std::stringstream ss;
      ss << "Reverse lookup of frame id " << frame_id_num << " failed!";
      throw tf2::LookupException(ss.str());
    }
    else
      return frameIDs_reverse[frame_id_num];
}

void BufferCore::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const
{
  if (!out)
  {
    return;
  }
  *out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                     lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                     "Tf has two or more unconnected trees.");
}

std::vector<std::string> BufferCore::getAllFrameNames() const
{
  std::vector<std::string> frames;
  _getFrameStrings(frames);
  return frames;
}

std::string BufferCore::allFramesAsString() const
{
  std::unique_lock<std::mutex> lock(frame_mutex_);
  return this->allFramesAsStringNoLock();
}

std::string BufferCore::allFramesAsStringNoLock() const
{
  std::stringstream mstream;

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCacheInterfacePtr frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if(  frame_ptr->getData(TimePointZero, temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
  }

  return mstream.str();
}

struct TimeAndFrameIDFrameComparator
{
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  CompactFrameID id;
};

tf2::TF2Error BufferCore::getLatestCommonTime(CompactFrameID target_id, CompactFrameID source_id, TimePoint & time, std::string * error_string) const
{
  // Error if one of the frames don't exist.
  if (source_id == 0 || target_id == 0) return tf2::TF2Error::LOOKUP_ERROR;

  if (source_id == target_id)
  {
    TimeCacheInterfacePtr cache = getFrame(source_id);
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    if (cache)
      time = cache->getLatestTimestamp();
    else
      time = TimePointZero;
    return tf2::TF2Error::NO_ERROR;
  }

  std::vector<P_TimeAndFrameID> lct_cache;

  // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  P_TimeAndFrameID temp;
  uint32_t depth = 0;
  TimePoint common_time = TimePoint::max();
  while (frame != 0)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if (latest.first != TimePointZero)
    {
      common_time = std::min(latest.first, common_time);
    }

    lct_cache.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      time = common_time;
      if (time == TimePoint::max())
      {
        time = TimePointZero;
      }
      return tf2::TF2Error::NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return tf2::TF2Error::LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
  frame = target_id;
  depth = 0;
  common_time = TimePoint::max();
  CompactFrameID common_parent = 0;
  while (true)
  {
    TimeCacheInterfacePtr cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      break;
    }

    if (latest.first != TimePointZero)
    {
      common_time = std::min(latest.first, common_time);
    }

    std::vector<P_TimeAndFrameID>::iterator it = std::find_if(lct_cache.begin(), lct_cache.end(), TimeAndFrameIDFrameComparator(latest.second));
    if (it != lct_cache.end()) // found a common parent
    {
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      time = common_time;
      if (time == TimePoint::max())
      {
        time = TimePointZero;
      }
      return tf2::TF2Error::NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsStringNoLock() << std::endl;
        *error_string = ss.str();
      }
      return tf2::TF2Error::LOOKUP_ERROR;
    }
  }

  if (common_parent == 0)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return tf2::TF2Error::CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
    for (; it != end; ++it)
    {
      if (it->first != TimePointZero)
      {
        common_time = std::min(common_time, it->first);
      }

      if (it->second == common_parent)
      {
        break;
      }
    }
  }

  if (common_time == TimePoint::max())
  {
    common_time = TimePointZero;
  }

  time = common_time;
  return tf2::TF2Error::NO_ERROR;
}

std::string BufferCore::allFramesAsYAML(TimePoint current_time) const
{
  std::stringstream mstream;
  std::unique_lock<std::mutex> lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() ==1)
    mstream <<"[]";

  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);

   //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    CompactFrameID cfid = CompactFrameID(counter);
    CompactFrameID frame_id_num;
    TimeCacheInterfacePtr cache = getFrame(cfid);
    if (!cache)
    {
      continue;
    }

    if(!cache->getData(TimePointZero, temp))
    {
      continue;
    }

    frame_id_num = temp.frame_id_;

    std::string authority = "no recorded authority";
    std::map<CompactFrameID, std::string>::const_iterator it = frame_authority_.find(cfid);
    if (it != frame_authority_.end()) {
      authority = it->second;
    }

    tf2::Duration dur1 = cache->getLatestTimestamp() - cache->getOldestTimestamp();
    tf2::Duration dur2 = tf2::Duration(std::chrono::microseconds(100));

    double rate;
    if (dur1 > dur2)
      rate = (cache->getListLength() * 1e9) / std::chrono::duration_cast<std::chrono::nanoseconds>(dur1).count();
    else
      rate = (cache->getListLength() * 1e9) / std::chrono::duration_cast<std::chrono::nanoseconds>(dur2).count();

    mstream << std::fixed; //fixed point notation
    mstream.precision(3); //3 decimal places
    mstream << frameIDs_reverse[cfid] << ": " << std::endl;
    mstream << "  parent: '" << frameIDs_reverse[frame_id_num] << "'" << std::endl;
    mstream << "  broadcaster: '" << authority << "'" << std::endl;
    mstream << "  rate: " << rate << std::endl;
    mstream << "  most_recent_transform: " << displayTimePoint(cache->getLatestTimestamp()) << std::endl;
    mstream << "  oldest_transform: " << displayTimePoint(cache->getOldestTimestamp()) << std::endl;
    if ( current_time !=TimePointZero ) {
      mstream << "  transform_delay: " << durationToSec(current_time - cache->getLatestTimestamp()) << std::endl;
    }
    mstream << "  buffer_length: " << durationToSec(cache->getLatestTimestamp() - cache->getOldestTimestamp()) << std::endl;
  }

  return mstream.str();
}

std::string BufferCore::allFramesAsYAML() const
{
  return this->allFramesAsYAML(TimePointZero);
}

TransformableCallbackHandle BufferCore::addTransformableCallback(const TransformableCallback& cb)
{
  std::unique_lock<std::mutex> lock(transformable_callbacks_mutex_);
  TransformableCallbackHandle handle = ++transformable_callbacks_counter_;
  while (!transformable_callbacks_.insert(std::make_pair(handle, cb)).second)
  {
    handle = ++transformable_callbacks_counter_;
  }

  return handle;
}

struct BufferCore::RemoveRequestByCallback
{
  RemoveRequestByCallback(TransformableCallbackHandle handle)
  : handle_(handle)
  {}

  bool operator()(const TransformableRequest& req)
  {
    return req.cb_handle == handle_;
  }

  TransformableCallbackHandle handle_;
};

void BufferCore::removeTransformableCallback(TransformableCallbackHandle handle)
{
  {
    std::unique_lock<std::mutex> lock(transformable_callbacks_mutex_);
    transformable_callbacks_.erase(handle);
  }

  {
    std::unique_lock<std::mutex> lock(transformable_requests_mutex_);
    V_TransformableRequest::iterator it = std::remove_if(transformable_requests_.begin(), transformable_requests_.end(), RemoveRequestByCallback(handle));
    transformable_requests_.erase(it, transformable_requests_.end());
  }
}

TransformableRequestHandle BufferCore::addTransformableRequest(TransformableCallbackHandle handle, const std::string& target_frame, const std::string& source_frame, TimePoint time)
{
  // shortcut if target == source
  if (target_frame == source_frame)
  {
    return 0;
  }

  // Even though we only modify transformable_requests_ at the end of the
  // method, we still need to take the lock near the beginning.  This is to
  // ensure that we don't have a TOCTTOU race between this method and
  // testTransformableRequests.  If the lock were only at the end of this
  // method, the race occurs like this:
  //
  // T1: addTransformableRequest, determines that needs to add to transformable_requests_
  // T2: in testTransformableRequests already, holding the lock
  // T1: blocked getting lock
  // T2: calls all callbacks for outstanding transforms (doesn't include the current one)
  // T2: unlocks
  // T1: gets lock, adds to list
  //
  // If nothing ever calls setTransform() again, then the callback for the
  // current request will never get called.  We fix this by holding the mutex
  // across most of this method.
  std::unique_lock<std::mutex> lock(transformable_requests_mutex_);

  TransformableRequest req;
  req.target_id = lookupFrameNumber(target_frame);
  req.source_id = lookupFrameNumber(source_frame);

  // First check if the request is already transformable.  If it is, return immediately
  if (canTransformInternal(req.target_id, req.source_id, time, 0))
  {
    return 0;
  }

  // Might not be transformable at all, ever (if it's too far in the past)
  if (req.target_id && req.source_id)
  {
    TimePoint latest_time;
    // TODO: This is incorrect, but better than nothing.  Really we want the latest time for
    // any of the frames
    getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
    if ((latest_time != TimePointZero) && (time + cache_time_ < latest_time))
    {
      return 0xffffffffffffffffULL;
    }
  }

  req.cb_handle = handle;
  req.time = time;
  req.request_handle = ++transformable_requests_counter_;
  if (req.request_handle == 0 || req.request_handle == 0xffffffffffffffffULL)
  {
    req.request_handle = 1;
  }

  if (req.target_id == 0)
  {
    req.target_string = target_frame;
  }

  if (req.source_id == 0)
  {
    req.source_string = source_frame;
  }

  transformable_requests_.push_back(req);

  return req.request_handle;
}

struct BufferCore::RemoveRequestByID
{
  RemoveRequestByID(TransformableRequestHandle handle)
  : handle_((TransformableCallbackHandle)handle)
  {}

  bool operator()(const TransformableRequest& req)
  {
    return req.request_handle == handle_;
  }

  TransformableCallbackHandle handle_;
};

void BufferCore::cancelTransformableRequest(TransformableRequestHandle handle)
{
  std::unique_lock<std::mutex> lock(transformable_requests_mutex_);
  V_TransformableRequest::iterator it = std::remove_if(transformable_requests_.begin(), transformable_requests_.end(), RemoveRequestByID(handle));

  if (it != transformable_requests_.end())
  {
    transformable_requests_.erase(it, transformable_requests_.end());
  }
}



// backwards compability for tf methods
bool BufferCore::_frameExists(const std::string& frame_id_str) const
{
  std::unique_lock<std::mutex> lock(frame_mutex_);
  return frameIDs_.count(frame_id_str) != 0;
}

bool BufferCore::_getParent(const std::string& frame_id, TimePoint time, std::string& parent) const
{

  std::unique_lock<std::mutex> lock(frame_mutex_);
  CompactFrameID frame_number = lookupFrameNumber(frame_id);
  TimeCacheInterfacePtr frame = getFrame(frame_number);

  if (! frame)
    return false;
      
  CompactFrameID parent_id = frame->getParent(time, NULL);
  if (parent_id == 0)
    return false;

  parent = lookupFrameString(parent_id);
  return true;
};

void BufferCore::_getFrameStrings(std::vector<std::string> & vec) const
{
  vec.clear();

  std::unique_lock<std::mutex> lock(frame_mutex_);

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frameIDs_reverse.size(); counter ++)
  {
    vec.push_back(frameIDs_reverse[counter]);
  }
  return;
}




void BufferCore::testTransformableRequests()
{
  std::unique_lock<std::mutex> lock(transformable_requests_mutex_);
  V_TransformableRequest::iterator it = transformable_requests_.begin();
  for (; it != transformable_requests_.end();)
  {
    TransformableRequest& req = *it;

    // One or both of the frames may not have existed when the request was originally made.
    if (req.target_id == 0)
    {
      req.target_id = lookupFrameNumber(req.target_string);
    }

    if (req.source_id == 0)
    {
      req.source_id = lookupFrameNumber(req.source_string);
    }

    TimePoint latest_time;
    bool do_cb = false;
    TransformableResult result = TransformAvailable;
    // TODO: This is incorrect, but better than nothing.  Really we want the latest time for
    // any of the frames
    getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
    if ((latest_time != TimePointZero) && (req.time + cache_time_ < latest_time))
    {
      do_cb = true;
      result = TransformFailure;
    }
    else if (canTransformInternal(req.target_id, req.source_id, req.time, 0))
    {
      do_cb = true;
      result = TransformAvailable;
    }

    if (do_cb)
    {
      {
        std::unique_lock<std::mutex> lock2(transformable_callbacks_mutex_);
        M_TransformableCallback::iterator it = transformable_callbacks_.find(req.cb_handle);
        if (it != transformable_callbacks_.end())
        {
          const TransformableCallback& cb = it->second;
          cb(req.request_handle, lookupFrameString(req.target_id), lookupFrameString(req.source_id), req.time, result);
          transformable_callbacks_.erase(req.cb_handle);
        }
      }

      if (transformable_requests_.size() > 1)
      {
        transformable_requests_[it - transformable_requests_.begin()] = transformable_requests_.back();
      }

      transformable_requests_.erase(transformable_requests_.end() - 1);

      // If we've removed the last element, then the iterator is invalid
      if (0u == transformable_requests_.size()) {
        it = transformable_requests_.end();
      }
    }
    else
    {
      ++it;
    }
  }

  // unlock before allowing possible user callbacks to avoid potential detadlock (#91)
  lock.unlock();
}


std::string BufferCore::_allFramesAsDot(TimePoint current_time) const
{
  std::stringstream mstream;
  mstream << "digraph G {" << std::endl;
  std::unique_lock<std::mutex> lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() == 1) {
    mstream <<"\"no tf data recieved\"";
  }
  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);

  for (unsigned int counter = 1; counter < frames_.size(); counter ++) // one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    TimeCacheInterfacePtr counter_frame = getFrame(counter);
    if (!counter_frame) {
      continue;
    }
    if(!counter_frame->getData(TimePointZero, temp)) {
      continue;
    } else {
      frame_id_num = temp.frame_id_;
    }
    std::string authority = "no recorded authority";
    std::map<unsigned int, std::string>::const_iterator it = frame_authority_.find(counter);
    if (it != frame_authority_.end())
      authority = it->second;

    tf2::Duration dur1 = counter_frame->getLatestTimestamp() - counter_frame->getOldestTimestamp();
    tf2::Duration dur2 = std::chrono::microseconds(100);

    double rate;
    if (dur1 > dur2)
      rate = (counter_frame->getListLength() * 1e9) / std::chrono::duration_cast<std::chrono::nanoseconds>(dur1).count();
    else
      rate = (counter_frame->getListLength() * 1e9) / std::chrono::duration_cast<std::chrono::nanoseconds>(dur2).count();

    mstream << std::fixed; //fixed point notation
    mstream.precision(3); //3 decimal places
    mstream << "\"" << frameIDs_reverse[frame_id_num] << "\"" << " -> "
            << "\"" << frameIDs_reverse[counter] << "\"" << "[label=\""
      //<< "Time: " << current_time.toSec() << "\\n"
            << "Broadcaster: " << authority << "\\n"
            << "Average rate: " << rate << " Hz\\n"
            << "Most recent transform: " << displayTimePoint(counter_frame->getLatestTimestamp()) <<" ";
    if (current_time != TimePointZero)
      mstream << "( "<<  durationToSec(current_time - counter_frame->getLatestTimestamp()) << " sec old)";
    mstream << "\\n"
      //    << "(time: " << getFrame(counter)->getLatestTimestamp().toSec() << ")\\n"
      //    << "Oldest transform: " << (current_time - getFrame(counter)->getOldestTimestamp()).toSec() << " sec old \\n"
      //    << "(time: " << (getFrame(counter)->getOldestTimestamp()).toSec() << ")\\n"
            << "Buffer length: " << durationToSec(counter_frame->getLatestTimestamp()-counter_frame->getOldestTimestamp()) << " sec\\n"
            <<"\"];" <<std::endl;
  }

  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    TimeCacheInterfacePtr counter_frame = getFrame(counter);
    if (!counter_frame) {
      if (current_time != TimePointZero) {
        mstream << "edge [style=invis];" <<std::endl;
        mstream << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n"
                << "\"Recorded at time: " << displayTimePoint(current_time) << "\"[ shape=plaintext ] ;\n "
                << "}" << "->" << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
      }
      continue;
    }
    if (counter_frame->getData(TimePointZero, temp)) {
      frame_id_num = temp.frame_id_;
    } else {
    	frame_id_num = 0;
    }

    if(frameIDs_reverse[frame_id_num]=="NO_PARENT")
    {
      mstream << "edge [style=invis];" <<std::endl;
      mstream << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n";
      if (current_time != TimePointZero)
        mstream << "\"Recorded at time: " << displayTimePoint(current_time) << "\"[ shape=plaintext ] ;\n ";
      mstream << "}" << "->" << "\"" << frameIDs_reverse[counter] << "\";" << std::endl;
    }
  }
  mstream << "}";
  return mstream.str();
}

std::string BufferCore::_allFramesAsDot() const
{
  return _allFramesAsDot(TimePointZero);
}

void BufferCore::_chainAsVector(const std::string & target_frame, TimePoint target_time, const std::string & source_frame, TimePoint source_time, const std::string& fixed_frame, std::vector<std::string>& output) const
{
  std::string error_string;

  output.clear(); //empty vector

  std::stringstream mstream;
  std::unique_lock<std::mutex> lock(frame_mutex_);

  TransformAccum accum;

  // Get source frame/time using getFrame
  CompactFrameID source_id = lookupFrameNumber(source_frame);
  CompactFrameID fixed_id = lookupFrameNumber(fixed_frame);
  CompactFrameID target_id = lookupFrameNumber(target_frame);

  std::vector<CompactFrameID> source_frame_chain;
  tf2::TF2Error retval = walkToTopParent(accum, source_time, fixed_id, source_id, &error_string, &source_frame_chain);

  if (retval != tf2::TF2Error::NO_ERROR)
  {
    switch (retval)
    {
    case tf2::TF2Error::CONNECTIVITY_ERROR:
      throw ConnectivityException(error_string);
    case tf2::TF2Error::EXTRAPOLATION_ERROR:
      throw ExtrapolationException(error_string);
    case tf2::TF2Error::LOOKUP_ERROR:
      throw LookupException(error_string);
    default:
      CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
      assert(0);
    }
  }
  if (source_time != target_time)
  {
    std::vector<CompactFrameID> target_frame_chain;
    retval = walkToTopParent(accum, target_time, target_id, fixed_id, &error_string, &target_frame_chain);

    if (retval != tf2::TF2Error::NO_ERROR)
    {
      switch (retval)
      {
      case tf2::TF2Error::CONNECTIVITY_ERROR:
        throw ConnectivityException(error_string);
      case tf2::TF2Error::EXTRAPOLATION_ERROR:
        throw ExtrapolationException(error_string);
      case tf2::TF2Error::LOOKUP_ERROR:
        throw LookupException(error_string);
      default:
        CONSOLE_BRIDGE_logError("Unknown error code: %d", retval);
        assert(0);
      }
    }
    int m = (int)target_frame_chain.size()-1;
    int n = (int)source_frame_chain.size()-1;
    for (; m >= 0 && n >= 0; --m, --n)
    {
      if (source_frame_chain[n] != target_frame_chain[m])
        break;
    }
    // Erase all duplicate items from frame_chain
    if (n > 0)
      source_frame_chain.erase(source_frame_chain.begin() + (n-1), source_frame_chain.end());

    if (m < target_frame_chain.size())
    {
      for (int i = 0; i <= m; ++i)
      {
        source_frame_chain.push_back(target_frame_chain[i]);
      }
    }
  }

  // Write each element of source_frame_chain as string
  for (unsigned int i = 0; i < source_frame_chain.size(); ++i)
  {
    output.push_back(lookupFrameString(source_frame_chain[i]));
 }
}


} // namespace tf2
