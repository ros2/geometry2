// Copyright 2008, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/** \author Tully Foote */

#ifndef TF2__BUFFER_CORE_H_
#define TF2__BUFFER_CORE_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "LinearMath/Transform.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/buffer_core_interface.h"
#include "tf2/exceptions.h"
#include "tf2/transform_storage.h"
#include "tf2/visibility_control.h"
#include "rcutils/logging_macros.h"

namespace tf2
{

typedef std::pair<TimePoint, CompactFrameID> P_TimeAndFrameID;
typedef uint64_t TransformableRequestHandle;

class TimeCacheInterface;
using TimeCacheInterfacePtr = std::shared_ptr<TimeCacheInterface>;

enum TransformableResult
{
  TransformAvailable,
  TransformFailure,
};

//!< The default amount of time to cache data in seconds
static constexpr Duration BUFFER_CORE_DEFAULT_CACHE_TIME = std::chrono::seconds(10);

/** \brief A Class which provides coordinate transforms between any two frames in a system.
 *
 * This class provides a simple interface to allow recording and lookup of
 * relationships between arbitrary frames of the system.
 *
 * libTF assumes that there is a tree of coordinate frame transforms which define the relationship between all coordinate frames.
 * For example your typical robot would have a transform from global to real world.  And then from base to hand, and from base to head.
 * But Base to Hand really is composed of base to shoulder to elbow to wrist to hand.
 * libTF is designed to take care of all the intermediate steps for you.
 *
 * Internal Representation
 * libTF will store frames with the parameters necessary for generating the transform into that frame from it's parent and a reference to the parent frame.
 * Frames are designated using an std::string
 * 0 is a frame without a parent (the top of a tree)
 * The positions of frames over time must be pushed in.
 *
 * All function calls which pass frame ids can potentially throw the exception tf::LookupException
 */
class BufferCore : public BufferCoreInterface
{
public:
  /************* Constants ***********************/
  //!< Maximum graph search depth (deeper graphs will be assumed to have loops)
  TF2_PUBLIC
  static const uint32_t MAX_GRAPH_DEPTH = 1000UL;

  /** Constructor
   * \param cache_time How long to keep a history of transforms in nanoseconds
   *
   */
  TF2_PUBLIC
  explicit BufferCore(tf2::Duration cache_time = BUFFER_CORE_DEFAULT_CACHE_TIME);

  TF2_PUBLIC
  virtual ~BufferCore(void);

  /** \brief Clear all data */
  TF2_PUBLIC
  void clear() override;

  /** \brief Add transform information to the tf data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
   * \return True unless an error occured
   */
  TF2_PUBLIC
  bool setTransform(
    const geometry_msgs::msg::TransformStamped & transform,
    const std::string & authority, bool is_static = false);

  /*********** Accessors *************/

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  TF2_PUBLIC
  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const TimePoint & time) const override;

  /** \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0 will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant in time.
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */

  TF2_PUBLIC
  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame, const TimePoint & target_time,
    const std::string & source_frame, const TimePoint & source_time,
    const std::string & fixed_frame) const override;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not nullptr
   * \return True if the transform is possible, false otherwise
   */
  TF2_PUBLIC
  bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const TimePoint & time, std::string * error_msg = nullptr) const override;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in time
   * \param error_msg A pointer to a string which will be filled with why the transform failed, if not nullptr
   * \return True if the transform is possible, false otherwise
   */
  TF2_PUBLIC
  bool canTransform(
    const std::string & target_frame, const TimePoint & target_time,
    const std::string & source_frame, const TimePoint & source_time,
    const std::string & fixed_frame, std::string * error_msg = nullptr) const override;

  /** \brief Get all frames that exist in the system.
   */
  TF2_PUBLIC
  std::vector<std::string> getAllFrameNames() const override;

  /** \brief A way to see what frames have been cached in yaml format
   * Useful for debugging tools
   */
  TF2_PUBLIC
  std::string allFramesAsYAML(TimePoint current_time) const;

  /** Backwards compatibility for #84
  */
  TF2_PUBLIC
  std::string allFramesAsYAML() const;

  /** \brief A way to see what frames have been cached
   * Useful for debugging
   */
  TF2_PUBLIC
  std::string allFramesAsString() const;

  using TransformableCallback = std::function<
    void (TransformableRequestHandle request_handle, const std::string & target_frame,
    const std::string & source_frame,
    TimePoint time, TransformableResult result)>;

  /// \brief Internal use only
  TF2_PUBLIC
  TransformableRequestHandle addTransformableRequest(
    const TransformableCallback & cb,
    const std::string & target_frame,
    const std::string & source_frame,
    TimePoint time);
  /// \brief Internal use only
  TF2_PUBLIC
  void cancelTransformableRequest(TransformableRequestHandle handle);


  // Tell the buffer that there are multiple threads servicing it.
  // This is useful for derived classes to know if they can block or not.
  TF2_PUBLIC
  void setUsingDedicatedThread(bool value) {using_dedicated_thread_ = value;}
  // Get the state of using_dedicated_thread_
  TF2_PUBLIC
  bool isUsingDedicatedThread() const {return using_dedicated_thread_;}


  /* Backwards compatability section for tf::Transformer you should not use these
   */

  /**@brief Check if a frame exists in the tree
   * @param frame_id_str The frame id in question  */
  TF2_PUBLIC
  bool _frameExists(const std::string & frame_id_str) const;

  /**@brief Fill the parent of a frame.
   * @param frame_id The frame id of the frame in question
   * @param time The timepoint of the frame in question
   * @param parent The reference to the string to fill the parent
   * Returns true unless "NO_PARENT" */
  TF2_PUBLIC
  bool _getParent(const std::string & frame_id, TimePoint time, std::string & parent) const;

  /** \brief A way to get a std::vector of available frame ids */
  TF2_PUBLIC
  void _getFrameStrings(std::vector<std::string> & ids) const;


  TF2_PUBLIC
  CompactFrameID _lookupFrameNumber(const std::string & frameid_str) const
  {
    return lookupFrameNumber(frameid_str);
  }
  TF2_PUBLIC
  CompactFrameID _lookupOrInsertFrameNumber(const std::string & frameid_str)
  {
    return lookupOrInsertFrameNumber(frameid_str);
  }

  TF2_PUBLIC
  tf2::TF2Error _getLatestCommonTime(
    CompactFrameID target_frame, CompactFrameID source_frame,
    TimePoint & time, std::string * error_string) const
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    return getLatestCommonTime(target_frame, source_frame, time, error_string);
  }

  TF2_PUBLIC
  CompactFrameID _validateFrameId(
    const char * function_name_arg,
    const std::string & frame_id) const
  {
    return validateFrameId(function_name_arg, frame_id);
  }

  /**@brief Get the duration over which this transformer will cache */
  TF2_PUBLIC
  tf2::Duration getCacheLength() {return cache_time_;}

  /** \brief Backwards compatabilityA way to see what frames have been cached
   * Useful for debugging
   */
  TF2_PUBLIC
  std::string _allFramesAsDot(TimePoint current_time) const;
  TF2_PUBLIC
  std::string _allFramesAsDot() const;

  /** \brief Backwards compatabilityA way to see what frames are in a chain
   * Useful for debugging
   */
  TF2_PUBLIC
  void _chainAsVector(
    const std::string & target_frame, TimePoint target_time,
    const std::string & source_frame, TimePoint source_time,
    const std::string & fixed_frame,
    std::vector<std::string> & output) const;

private:
  /******************** Internal Storage ****************/

  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  typedef std::vector<TimeCacheInterfacePtr> V_TimeCacheInterface;
  V_TimeCacheInterface frames_;

  /** \brief A mutex to protect testing and allocating new frames on the above vector. */
  mutable std::mutex frame_mutex_;

  /** \brief A map from string frame ids to CompactFrameID */
  typedef std::unordered_map<std::string, CompactFrameID> M_StringToCompactFrameID;
  M_StringToCompactFrameID frameIDs_;
  /** \brief A map from CompactFrameID frame_id_numbers to string for debugging and output */
  std::vector<std::string> frameIDs_reverse_;
  /** \brief A map to lookup the most recent authority for a given frame */
  std::map<CompactFrameID, std::string> frame_authority_;


  /// How long to cache transform history
  tf2::Duration cache_time_;

  typedef uint32_t TransformableCallbackHandle;

  typedef std::unordered_map<TransformableCallbackHandle,
      TransformableCallback> M_TransformableCallback;
  M_TransformableCallback transformable_callbacks_;
  uint32_t transformable_callbacks_counter_;
  std::mutex transformable_callbacks_mutex_;

  struct TransformableRequest
  {
    TimePoint time;
    TransformableRequestHandle request_handle;
    TransformableCallbackHandle cb_handle;
    CompactFrameID target_id;
    CompactFrameID source_id;
    std::string target_string;
    std::string source_string;
  };
  typedef std::vector<TransformableRequest> V_TransformableRequest;
  V_TransformableRequest transformable_requests_;
  std::mutex transformable_requests_mutex_;
  uint64_t transformable_requests_counter_;

  bool using_dedicated_thread_;

  /************************* Internal Functions ****************************/

  /** \brief A way to see what frames have been cached
   * Useful for debugging. Use this call internally.
   */
  std::string allFramesAsStringNoLock() const;

  bool setTransformImpl(
    const tf2::Transform & transform_in, const std::string frame_id,
    const std::string child_frame_id, const TimePoint stamp,
    const std::string & authority, bool is_static);
  void lookupTransformImpl(
    const std::string & target_frame, const std::string & source_frame,
    const TimePoint & time_in, tf2::Transform & transform, TimePoint & time_out) const;

  void lookupTransformImpl(
    const std::string & target_frame, const TimePoint & target_time,
    const std::string & source_frame, const TimePoint & source_time,
    const std::string & fixed_frame, tf2::Transform & transform, TimePoint & time_out) const;

  /** \brief An accessor to get a frame.
   * \param c_frame_id The frameID of the desired Reference Frame
   */
  TimeCacheInterfacePtr getFrame(CompactFrameID c_frame_id) const;

  TimeCacheInterfacePtr allocateFrame(CompactFrameID cfid, bool is_static);

  /** \brief Validate a frame ID format and look up its CompactFrameID.
    *   For invalid cases, produce an message.
    * \param function_name_arg string to print out in the message,
    *   the current function and argument name being validated
    * \param frame_id name of the tf frame to validate
    * \param[out] error_msg if non-nullptr, fill with produced error messaging.
    *   Otherwise messages are logged as warning.
    * \return The CompactFrameID of the frame or 0 if not found.
    */
  CompactFrameID validateFrameId(
    const char * function_name_arg,
    const std::string & frame_id,
    std::string * error_msg) const;

  /** \brief Validate a frame ID format and look it up its compact ID.
    *   Raise an exception for invalid cases.
    * \param function_name_arg string to print out in the exception,
    *   the current function and argument name being validated
    * \param frame_id name of the tf frame to validate
    * \return The CompactFrameID of the existing frame.
    * \throws InvalidArgumentException if the frame_id string has an invalid format
    * \throws LookupException if frame_id did not exist
    */
  CompactFrameID validateFrameId(
    const char * function_name_arg,
    const std::string & frame_id) const;

  /// String to number for frame lookup. Returns 0 if the frame was not found.
  CompactFrameID lookupFrameNumber(const std::string & frameid_str) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookupOrInsertFrameNumber(const std::string & frameid_str);

  /// Number to string frame lookup may throw LookupException if number invalid
  const std::string & lookupFrameString(CompactFrameID frame_id_num) const;

  void createConnectivityErrorString(
    CompactFrameID source_frame, CompactFrameID target_frame,
    std::string * out) const;

  /**@brief Return the latest rostime which is common across the spanning set
   * zero if fails to cross */
  tf2::TF2Error getLatestCommonTime(
    CompactFrameID target_frame, CompactFrameID source_frame,
    TimePoint & time, std::string * error_string) const;

  /**@brief Traverse the transform tree. If frame_chain is not nullptr, store the traversed frame tree in vector frame_chain.
   * */
  template<typename F>
  tf2::TF2Error walkToTopParent(
    F & f, TimePoint time, CompactFrameID target_id,
    CompactFrameID source_id, std::string * error_string,
    std::vector<CompactFrameID> * frame_chain) const;

  void testTransformableRequests();

  // Actual implementation to walk the transform tree and find out if a transform exists.
  bool canTransformInternal(
    CompactFrameID target_id, CompactFrameID source_id,
    const TimePoint & time, std::string * error_msg) const;
};
}  // namespace tf2

#endif  // TF2__BUFFER_CORE_H_
