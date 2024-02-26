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

/** \author Josh Faust */

#ifndef TF2_ROS__MESSAGE_FILTER_H_
#define TF2_ROS__MESSAGE_FILTER_H_

#include <algorithm>
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <ratio>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "message_filters/connection.h"
#include "message_filters/message_traits.h"
#include "message_filters/simple_filter.h"
#include "tf2/buffer_core_interface.h"
#include "tf2/time.h"
#include "tf2_ros/async_buffer_interface.h"
#include "tf2_ros/buffer.h"

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

#define TF2_ROS_MESSAGEFILTER_DEBUG(fmt, ...) \
  RCUTILS_LOG_DEBUG_NAMED( \
    "tf2_ros_message_filter", \
    std::string(std::string("MessageFilter [target=%s]: ") + std::string(fmt)).c_str(), \
    getTargetFramesString().c_str(), __VA_ARGS__)

#define TF2_ROS_MESSAGEFILTER_WARN(fmt, ...) \
  RCUTILS_LOG_WARN_NAMED( \
    "tf2_ros_message_filter", \
    std::string(std::string("MessageFilter [target=%s]: ") + std::string(fmt)).c_str(), \
    getTargetFramesString().c_str(), __VA_ARGS__)

namespace tf2_ros
{

namespace filter_failure_reasons
{
enum FilterFailureReason
{
  // NOTE when adding new values, do not explicitly assign a number. See FilterFailureReasonCount

  /// The message buffer overflowed, and this message was pushed off the back of the queue, but the
  // reason it was unable to be transformed is unknown.
  Unknown,
  /// The timestamp on the message is earlier than all the data in the transform cache
  OutTheBack,
  /// The frame_id on the message is empty
  EmptyFrameID,
  /// No transform found
  NoTransformFound,
  /// Queue size full
  QueueFull,
  /// Max enum value for iteration, keep it at the end of the enum
  FilterFailureReasonCount,
};

}  // namespace filter_failure_reasons

static std::string get_filter_failure_reason_string(
  filter_failure_reasons::FilterFailureReason reason)
{
  switch (reason) {
    case filter_failure_reasons::OutTheBack:
      return
        "the timestamp on the message is earlier than all the data in the transform cache";
    case filter_failure_reasons::EmptyFrameID:
      return "the frame id of the message is empty";
    case filter_failure_reasons::NoTransformFound:
      return "did not find a valid transform, this usually happens at startup ...";
    case filter_failure_reasons::QueueFull:
      return "discarding message because the queue is full";
    case filter_failure_reasons::Unknown:  // fallthrough
    default:
      return "unknown";
  }
}

typedef filter_failure_reasons::FilterFailureReason FilterFailureReason;

class MessageFilterBase
{
public:
  typedef std::vector<std::string> V_string;

  virtual ~MessageFilterBase() {}
  virtual void clear() = 0;
  virtual void setTargetFrame(const std::string & target_frame) = 0;
  virtual void setTargetFrames(const V_string & target_frames) = 0;
  virtual void setTolerance(const rclcpp::Duration & tolerance) = 0;
};

/**
 * \brief Follows the patterns set by the message_filters package to implement a filter which only passes messages through once there is transform data available
 *
 * The callbacks used in this class are of the same form as those used by rclcpp's message callbacks.
 *
 * MessageFilter is templated on a message type.
 *
 * \section example_usage Example Usage
 *
 * If you want to hook a MessageFilter into a ROS topic:
 \verbatim
   message_filters::Subscriber<MessageType> sub(node_, "topic", 10);
   tf::MessageFilter<MessageType> tf_filter(sub, tf_listener_, "/map", 10);
   tf_filter.registerCallback(&MyClass::myCallback, this);
 \endverbatim
 */
template<class M, class BufferT = tf2_ros::Buffer>
class MessageFilter : public MessageFilterBase, public message_filters::SimpleFilter<M>
{
public:
  using MConstPtr = std::shared_ptr<M const>;
  typedef message_filters::MessageEvent<M const> MEvent;

  /**
   * \brief Constructor
   *
   * \param buffer The buffer this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node The ros2 node to use for logging and clock operations
   * \param buffer_timeout The timeout duration after requesting transforms from the buffer.
   */
  template<typename TimeRepT = int64_t, typename TimeT = std::nano>
  MessageFilter(
    BufferT & buffer, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::Node::SharedPtr & node,
    std::chrono::duration<TimeRepT, TimeT> buffer_timeout =
    std::chrono::duration<TimeRepT, TimeT>::max())
  : MessageFilter(
      buffer, target_frame, queue_size, node->get_node_logging_interface(),
      node->get_node_clock_interface(), buffer_timeout)
  {
    static_assert(
      std::is_base_of<tf2::BufferCoreInterface, BufferT>::value,
      "Buffer type must implement tf2::BufferCoreInterface");
    static_assert(
      std::is_base_of<tf2_ros::AsyncBufferInterface, BufferT>::value,
      "Buffer type must implement tf2_ros::AsyncBufferInterface");
  }

  /**
   * \brief Constructor
   *
   * \param buffer The buffer this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node_logging The logging interface to use for any log messages
   * \param node_clock The clock interface to use to get the node clock
   * \param buffer_timeout The timeout duration after requesting transforms from the buffer.
   */
  template<typename TimeRepT = int64_t, typename TimeT = std::nano>
  MessageFilter(
    BufferT & buffer, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
    std::chrono::duration<TimeRepT, TimeT> buffer_timeout =
    std::chrono::duration<TimeRepT, TimeT>::max())
  : node_logging_(node_logging),
    node_clock_(node_clock),
    buffer_(buffer),
    queue_size_(queue_size),
    buffer_timeout_(buffer_timeout)
  {
    init();
    setTargetFrame(target_frame);
  }

  /**
   * \brief Constructor
   *
   * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
   * \param buffer The buffer this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node The ros2 node to use for logging and clock operations
   * \param buffer_timeout The timeout duration after requesting transforms from the buffer.
   */
  template<class F, typename TimeRepT = int64_t, typename TimeT = std::nano>
  MessageFilter(
    F & f, BufferT & buffer, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::Node::SharedPtr & node,
    std::chrono::duration<TimeRepT, TimeT> buffer_timeout =
    std::chrono::duration<TimeRepT, TimeT>::max())
  : MessageFilter(
      f, buffer, target_frame, queue_size, node->get_node_logging_interface(),
      node->get_node_clock_interface(), buffer_timeout)
  {
  }

  /**
   * \brief Constructor
   *
   * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
   * \param buffer The buffer this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node_logging The logging interface to use for any log messages
   * \param node_clock The clock interface to use to get the node clock
   * \param buffer_timeout The timeout duration after requesting transforms from the buffer.
   */
  template<class F, typename TimeRepT = int64_t, typename TimeT = std::nano>
  MessageFilter(
    F & f, BufferT & buffer, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock,
    std::chrono::duration<TimeRepT, TimeT> buffer_timeout =
    std::chrono::duration<TimeRepT, TimeT>::max())
  : node_logging_(node_logging),
    node_clock_(node_clock),
    buffer_(buffer),
    queue_size_(queue_size),
    buffer_timeout_(buffer_timeout)
  {
    init();
    setTargetFrame(target_frame);
    connectInput(f);
  }

  /**
   * \brief Connect this filter's input to another filter's output.  If this filter is already connected, disconnects first.
   */
  template<class F>
  void connectInput(F & f)
  {
    message_connection_.disconnect();
    message_connection_ = f.registerCallback(&MessageFilter::incomingMessage, this);
  }

  /**
   * \brief Destructor
   */
  ~MessageFilter()
  {
    message_connection_.disconnect();
    clear();

    TF2_ROS_MESSAGEFILTER_DEBUG(
      "Successful Transforms: %llu, Discarded due to age: %llu, Transform messages received: %llu, "
      "Messages received: %llu, Total dropped: %llu",
      static_cast<uint64_t>(successful_transform_count_),
      static_cast<uint64_t>(failed_out_the_back_count_),
      static_cast<uint64_t>(transform_message_count_),
      static_cast<uint64_t>(incoming_message_count_),
      static_cast<uint64_t>(dropped_message_count_));
  }

  /**
   * \brief Set the frame you need to be able to transform to before getting a message callback
   */
  void setTargetFrame(const std::string & target_frame)
  {
    V_string frames;
    frames.push_back(target_frame);
    setTargetFrames(frames);
  }

  /**
   * \brief Set the frames you need to be able to transform to before getting a message callback
   */
  void setTargetFrames(const V_string & target_frames)
  {
    std::unique_lock<std::mutex> frames_lock(target_frames_mutex_);

    target_frames_.resize(target_frames.size());
    std::transform(
      target_frames.begin(), target_frames.end(),
      target_frames_.begin(), this->stripSlash);
    expected_success_count_ = target_frames_.size() * (time_tolerance_.nanoseconds() ? 2 : 1);

    std::stringstream ss;
    for (V_string::iterator it = target_frames_.begin(); it != target_frames_.end(); ++it) {
      ss << *it << " ";
    }
    target_frames_string_ = ss.str();
  }

  /**
   * \brief Get the target frames as a string for debugging
   */
  std::string getTargetFramesString()
  {
    std::unique_lock<std::mutex> frames_lock(target_frames_mutex_);
    return target_frames_string_;
  }

  /**
   * \brief Set the required tolerance for the notifier to return true
   */
  void setTolerance(const rclcpp::Duration & tolerance)
  {
    std::unique_lock<std::mutex> frames_lock(target_frames_mutex_);
    time_tolerance_ = tolerance;
    expected_success_count_ = target_frames_.size() * (time_tolerance_.nanoseconds() ? 2 : 1);
  }

  /**
   * \brief Clear any messages currently in the queue
   */
  void clear()
  {
    {
      std::unique_lock<std::mutex> lock(ts_futures_mutex_);
      for (auto & kv : ts_futures_) {
        buffer_.cancel(kv.second);
      }
      ts_futures_.clear();
    }

    std::unique_lock<std::mutex> unique_lock(messages_mutex_);

    TF2_ROS_MESSAGEFILTER_DEBUG("%s", "Cleared");

    messages_.clear();

    warned_about_empty_frame_id_ = false;
  }

  void add(const MEvent & evt)
  {
    if (target_frames_.empty()) {
      return;
    }

    namespace mt = message_filters::message_traits;
    const MConstPtr & message = evt.getMessage();
    std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
    rclcpp::Time stamp = mt::TimeStamp<M>::value(*message);

    if (frame_id.empty()) {
      messageDropped(evt, filter_failure_reasons::EmptyFrameID);
      return;
    }

    std::vector<std::tuple<uint64_t, tf2::TimePoint, std::string>> wait_params;
    // iterate through the target frames and add requests for each of them
    MessageInfo info;
    info.handles.reserve(expected_success_count_);
    {
      V_string target_frames_copy;
      // Copy target_frames_ to avoid deadlock from #79
      {
        std::unique_lock<std::mutex> frames_lock(target_frames_mutex_);
        target_frames_copy = target_frames_;
      }

      V_string::iterator it = target_frames_copy.begin();
      V_string::iterator end = target_frames_copy.end();
      for (; it != end; ++it) {
        const std::string & target_frame = *it;
        wait_params.emplace_back(
          next_handle_index_, tf2_ros::fromRclcpp(stamp), target_frame);
        info.handles.push_back(next_handle_index_++);

        if (time_tolerance_.nanoseconds()) {
          wait_params.emplace_back(
            next_handle_index_,
            tf2_ros::fromRclcpp(stamp + time_tolerance_),
            target_frame);
          info.handles.push_back(next_handle_index_++);
        }
      }
    }

    {
      // Keep a lock on the messages
      std::unique_lock<std::mutex> unique_lock(messages_mutex_);

      // If this message is about to push us past our queue size, erase the oldest message
      if (queue_size_ != 0 && messages_.size() + 1 > queue_size_) {
        ++dropped_message_count_;
        const MessageInfo & front = messages_.front();
        TF2_ROS_MESSAGEFILTER_DEBUG(
          "Removed oldest message because buffer is full, count now %d (frame_id=%s, stamp=%f)",
          messages_.size(),
          (mt::FrameId<M>::value(*front.event.getMessage())).c_str(),
          mt::TimeStamp<M>::value(*front.event.getMessage()).seconds());

        messageDropped(front.event, filter_failure_reasons::QueueFull);

        messages_.pop_front();
      }

      // Add the message to our list
      info.event = evt;
      messages_.push_back(info);
    }

    TF2_ROS_MESSAGEFILTER_DEBUG(
      "Added message in frame %s at time %.3f, count now %d",
      frame_id.c_str(), stamp.seconds(), messages_.size());
    ++incoming_message_count_;

    for (const auto & param : wait_params) {
      const auto & handle = std::get<0>(param);
      const auto & stamp = std::get<1>(param);
      const auto & target_frame = std::get<2>(param);
      tf2_ros::TransformStampedFuture future = buffer_.waitForTransform(
        target_frame,
        frame_id,
        stamp,
        buffer_timeout_,
        std::bind(&MessageFilter::transformReadyCallback, this, std::placeholders::_1, handle));

      // If handle of future is 0 or 0xffffffffffffffffULL, waitForTransform have already called
      // the callback.
      if (0 != future.getHandle() && 0xffffffffffffffffULL != future.getHandle()) {
        std::unique_lock<std::mutex> lock(ts_futures_mutex_);
        ts_futures_.insert({handle, std::move(future)});
      }
    }
  }

  /**
   * \brief Manually add a message into this filter.
   * \note If the message (or any other messages in the queue) are immediately transformable this will immediately call through to the output callback, possibly
   * multiple times
   */
  void add(const MConstPtr & message)
  {
    auto t = node_clock_->get_clock()->now();
    add(MEvent(message, t));
  }

  /**
   * \brief Register a callback to be called when a message is about to be dropped
   * \param callback The callback to call
   */
#if 0
  message_filters::Connection registerFailureCallback(const FailureCallback & callback)
  {
    message_connection_failure.disconnect();
    message_connection_failure = this->registerCallback(callback, this);
  }
#endif

  virtual void setQueueSize(uint32_t new_queue_size)
  {
    queue_size_ = new_queue_size;
  }

  virtual uint32_t getQueueSize()
  {
    return queue_size_;
  }

private:
  void init()
  {
    successful_transform_count_ = 0;
    failed_out_the_back_count_ = 0;
    transform_message_count_ = 0;
    incoming_message_count_ = 0;
    dropped_message_count_ = 0;
    time_tolerance_ = rclcpp::Duration(0, 0);
    warned_about_empty_frame_id_ = false;
    expected_success_count_ = 1;
  }

  void transformReadyCallback(const tf2_ros::TransformStampedFuture & future, const uint64_t handle)
  {
    namespace mt = message_filters::message_traits;

    MEvent saved_event;
    bool event_found = false;

    {
      std::unique_lock<std::mutex> lock(ts_futures_mutex_);
      auto iter = ts_futures_.find(handle);
      if (iter != ts_futures_.end()) {
        ts_futures_.erase(iter);
      }
    }

    {
      // We will be accessing and mutating messages now, require unique lock
      std::unique_lock<std::mutex> lock(messages_mutex_);

      // find the message this request is associated with
      typename L_MessageInfo::iterator msg_it = messages_.begin();
      typename L_MessageInfo::iterator msg_end = messages_.end();

      for (; msg_it != msg_end; ++msg_it) {
        MessageInfo & info = *msg_it;
        auto handle_it = std::find(info.handles.begin(), info.handles.end(), handle);
        if (handle_it != info.handles.end()) {
          // found msg_it
          ++info.success_count;
          if (info.success_count >= expected_success_count_) {
            saved_event = msg_it->event;
            messages_.erase(msg_it);
            event_found = true;
          }
          break;
        }
      }
    }

    if (!event_found) {
      return;
    }

    bool can_transform = true;
    const MConstPtr & message = saved_event.getMessage();
    std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
    rclcpp::Time stamp = mt::TimeStamp<M>::value(*message);

    bool transform_available = true;
    FilterFailureReason error = filter_failure_reasons::Unknown;
    try {
      future.get();
    } catch (...) {
      transform_available = false;
      error = filter_failure_reasons::OutTheBack;
    }

    if (transform_available) {
      std::unique_lock<std::mutex> frames_lock(target_frames_mutex_);
      // make sure we can still perform all the necessary transforms
      typename V_string::iterator it = target_frames_.begin();
      typename V_string::iterator end = target_frames_.end();
      for (; it != end; ++it) {
        const std::string & target = *it;
        if (!buffer_.canTransform(target, frame_id, tf2_ros::fromRclcpp(stamp), NULL)) {
          can_transform = false;
          break;
        }

        if (time_tolerance_.nanoseconds()) {
          if (!buffer_.canTransform(
              target, frame_id,
              tf2_ros::fromRclcpp(stamp + time_tolerance_), NULL))
          {
            can_transform = false;
            break;
          }
        }
      }
    } else {
      can_transform = false;
    }

    if (can_transform) {
      TF2_ROS_MESSAGEFILTER_DEBUG(
        "Message ready in frame %s at time %.3f, count now %d",
        frame_id.c_str(), stamp.seconds(), messages_.size());

      ++successful_transform_count_;
      messageReady(saved_event);
    } else {
      ++dropped_message_count_;

      TF2_ROS_MESSAGEFILTER_DEBUG(
        "Discarding message in frame %s at time %.3f, count now %d",
        frame_id.c_str(), stamp.seconds(), messages_.size());
      messageDropped(saved_event, error);
    }
  }

  /**
   * \brief Callback that happens when we receive a message on the message topic
   */
  void incomingMessage(const message_filters::MessageEvent<M const> & evt)
  {
    add(evt);
  }

  void checkFailures()
  {
    if (!next_failure_warning_.nanoseconds()) {
      next_failure_warning_ = node_clock_->get_clock()->now() + rclcpp::Duration(15, 0);
    }

    if (node_clock_->get_clock()->now() >= next_failure_warning_) {
      if (incoming_message_count_ - messages_.size() == 0) {
        return;
      }

      double dropped_pct = static_cast<double>(dropped_message_count_) /
        static_cast<double>(incoming_message_count_ - messages_.size());
      if (dropped_pct > 0.95) {
        TF2_ROS_MESSAGEFILTER_WARN(
          "Dropped %.2f%% of messages so far. Please turn the "
          "[tf2_ros_message_filter.message_notifier] rosconsole logger to DEBUG for more "
          "information.",
          dropped_pct * 100);
        next_failure_warning_ = node_clock_->get_clock()->now() + rclcpp::Duration(60, 0);

        if (static_cast<double>(failed_out_the_back_count_) /
          static_cast<double>(dropped_message_count_) > 0.5)
        {
          TF2_ROS_MESSAGEFILTER_WARN(
            "  The majority of dropped messages were due to messages growing older than the TF "
            "cache time.  The last message's timestamp was: %f, and the last frame_id was: %s",
            last_out_the_back_stamp_.seconds(), last_out_the_back_frame_.c_str());
        }
      }
    }
  }

  // TODO(clalancette): reenable this once we have underlying support for callback queues
#if 0
  struct CBQueueCallback : public ros::CallbackInterface
  {
    CBQueueCallback(
      MessageFilter * filter, const MEvent & event, bool success, FilterFailureReason reason)
    : event_(event),
      filter_(filter),
      reason_(reason),
      success_(success)
    {}


    virtual CallResult call()
    {
      if (success_) {
        filter_->signalMessage(event_);
      } else {
        filter_->signalFailure(event_, reason_);
      }

      return Success;
    }

private:
    MEvent event_;
    MessageFilter * filter_;
    FilterFailureReason reason_;
    bool success_;
  };
#endif

  void messageDropped(const MEvent & evt, FilterFailureReason reason)
  {
    // TODO(clalancette): reenable this once we have underlying support for callback queues
#if 0
    if (callback_queue_) {
      ros::CallbackInterfacePtr cb(new CBQueueCallback(this, evt, false, reason));
      callback_queue_->addCallback(cb, (uint64_t)this);
    } else {}
#endif
    {
      signalFailure(evt, reason);
    }
  }

  void messageReady(const MEvent & evt)
  {
    // TODO(clalancette): reenable this once we have underlying support for callback queues
#if 0
    if (callback_queue_) {
      ros::CallbackInterfacePtr cb(new CBQueueCallback(
          this, evt, true, filter_failure_reasons::Unknown));
      callback_queue_->addCallback(cb, (uint64_t)this);
    } else {}
#endif
    {
      this->signalMessage(evt);
    }
  }

  void signalFailure(const MEvent & evt, FilterFailureReason reason)
  {
    namespace mt = message_filters::message_traits;
    const MConstPtr & message = evt.getMessage();
    std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
    rclcpp::Time stamp = mt::TimeStamp<M>::value(*message);
    RCLCPP_INFO(
      node_logging_->get_logger(),
      "Message Filter dropping message: frame '%s' at time %.3f for reason '%s'",
      frame_id.c_str(), stamp.seconds(), get_filter_failure_reason_string(reason).c_str());
  }

  static std::string stripSlash(const std::string & in)
  {
    if (!in.empty() && (in[0] == '/')) {
      std::string out = in;
      out.erase(0, 1);
      return out;
    }

    return in;
  }

  ///< The node logging interface to use for any log messages
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  ///< The node clock interface to use to get the clock to use
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  ///< The Transformer used to determine if transformation data is available
  BufferT & buffer_;
  ///< The frames we need to be able to transform to before a message is ready
  V_string target_frames_;
  std::string target_frames_string_;
  ///< A mutex to protect access to the target_frames_ list and target_frames_string.
  std::mutex target_frames_mutex_;
  ///< The maximum number of messages we queue up
  uint32_t queue_size_;

  uint64_t next_handle_index_ = 0;
  struct MessageInfo
  {
    MessageInfo()
    : success_count(0) {}

    MEvent event;
    std::vector<uint64_t> handles;
    uint64_t success_count;
  };
  typedef std::list<MessageInfo> L_MessageInfo;
  L_MessageInfo messages_;

  ///< The mutex used for locking message list operations
  std::mutex messages_mutex_;
  uint64_t expected_success_count_;

  bool warned_about_empty_frame_id_;

  uint64_t successful_transform_count_;
  uint64_t failed_out_the_back_count_;
  uint64_t transform_message_count_;
  uint64_t incoming_message_count_;
  uint64_t dropped_message_count_;

  rclcpp::Time last_out_the_back_stamp_;
  std::string last_out_the_back_frame_;

  rclcpp::Time next_failure_warning_;

  ///< Provide additional tolerance on time for messages which are stamped
  // but can have associated duration
  rclcpp::Duration time_tolerance_ = rclcpp::Duration(0, 0);

  message_filters::Connection message_connection_;
  message_filters::Connection message_connection_failure;

  // Timeout duration when calling the buffer method 'waitForTransform'
  tf2::Duration buffer_timeout_;

  ///< The mutex used for locking TransformStampedFuture map operations
  std::mutex ts_futures_mutex_;

  ///< Store the TransformStampedFuture returned by 'waitForTransform',
  // to clear the callback in the Buffer if MessageFiltered object is destroyed.
  std::unordered_map<uint64_t, tf2_ros::TransformStampedFuture> ts_futures_;
};
}  // namespace tf2_ros

#endif  // TF2_ROS__MESSAGE_FILTER_H_
