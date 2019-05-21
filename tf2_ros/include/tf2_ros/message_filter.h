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

#ifndef TF2_ROS_MESSAGE_FILTER_H
#define TF2_ROS_MESSAGE_FILTER_H

#include <list>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/connection.h>
#include <message_filters/message_traits.h>
#include <message_filters/simple_filter.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>

#define TF2_ROS_MESSAGEFILTER_DEBUG(fmt, ...) \
  RCUTILS_LOG_DEBUG_NAMED("tf2_ros_message_filter", \
    std::string(std::string("MessageFilter [target=%s]: ") + std::string(fmt)).c_str(), \
    getTargetFramesString().c_str(), __VA_ARGS__)

#define TF2_ROS_MESSAGEFILTER_WARN(fmt, ...) \
  RCUTILS_LOG_WARN_NAMED("tf2_ros_message_filter", \
    std::string(std::string("MessageFilter [target=%s]: ") + std::string(fmt)).c_str(), \
    getTargetFramesString().c_str(), __VA_ARGS__)

namespace tf2_ros
{

namespace filter_failure_reasons
{
enum FilterFailureReason
{
  /// The message buffer overflowed, and this message was pushed off the back of the queue, but the reason it was unable to be transformed is unknown.
  Unknown,
  /// The timestamp on the message is more than the cache length earlier than the newest data in the transform cache
  OutTheBack,
  /// The frame_id on the message is empty
  EmptyFrameID,
};
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
template<class M>
class MessageFilter : public MessageFilterBase, public message_filters::SimpleFilter<M>
{
public:
  using MConstPtr = std::shared_ptr<M const>;
  typedef message_filters::MessageEvent<M const> MEvent;
  // typedef std::function<void(const MConstPtr&, FilterFailureReason)> FailureCallback;

  // If you hit this assert your message does not have a header, or does not have the HasHeader trait defined for it
  // Actually, we need to check that the message has a header, or that it
  // has the FrameId and Stamp traits. However I don't know how to do that
  // so simply commenting out for now.
  // ROS_STATIC_ASSERT(ros::message_traits::HasHeader<M>::value);

  /**
   * \brief Constructor
   *
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node The ros2 node to use for logging and clock operations
   */
  MessageFilter(
    tf2::BufferCore & bc, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::Node::SharedPtr & node)
  : MessageFilter(bc, target_frame, queue_size, node->get_node_logging_interface(),
      node->get_node_clock_interface())
  {
  }

  /**
   * \brief Constructor
   *
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node_logging The logging interface to use for any log messages
   * \param node_clock The clock interface to use to get the node clock
   */
  MessageFilter(
    tf2::BufferCore & bc, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock)
  : node_logging_(node_logging),
    node_clock_(node_clock),
    bc_(bc),
    queue_size_(queue_size)
  {
    init();
    setTargetFrame(target_frame);
  }

  /**
   * \brief Constructor
   *
   * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node The ros2 node to use for logging and clock operations
   */
  template<class F>
  MessageFilter(
    F & f, tf2::BufferCore & bc, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::Node::SharedPtr & node)
  : MessageFilter(f, bc, target_frame, queue_size, node->get_node_logging_interface(),
      node->get_node_clock_interface())
  {
  }

  /**
   * \brief Constructor
   *
   * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param node_logging The logging interface to use for any log messages
   * \param node_clock The clock interface to use to get the node clock
   */
  template<class F>
  MessageFilter(
    F & f, tf2::BufferCore & bc, const std::string & target_frame, uint32_t queue_size,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock)
  : bc_(bc),
    queue_size_(queue_size),
    node_logging_(node_logging),
    node_clock_(node_clock)
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
      "Successful Transforms: %llu, Discarded due to age: %llu, Transform messages received: %llu, Messages received: %llu, Total dropped: %llu",
      static_cast<long long unsigned int>(successful_transform_count_),
      static_cast<long long unsigned int>(failed_out_the_back_count_),
      static_cast<long long unsigned int>(transform_message_count_),
      static_cast<long long unsigned int>(incoming_message_count_),
      static_cast<long long unsigned int>(dropped_message_count_));
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
    std::transform(target_frames.begin(), target_frames.end(),
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
    std::unique_lock<std::mutex> unique_lock(messages_mutex_);

    TF2_ROS_MESSAGEFILTER_DEBUG("%s", "Cleared");

    bc_.removeTransformableCallback(callback_handle_);
    callback_handle_ = bc_.addTransformableCallback(std::bind(&MessageFilter::transformable,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5));

    messages_.clear();
    message_count_ = 0;

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
        tf2::TransformableRequestHandle handle = bc_.addTransformableRequest(callback_handle_,
            target_frame, frame_id, tf2::timeFromSec(
              stamp.seconds()));
        if (handle == 0xffffffffffffffffULL) {
          // never transformable
          messageDropped(evt, filter_failure_reasons::OutTheBack);
          return;
        } else if (handle == 0) {
          ++info.success_count;
        } else {
          info.handles.push_back(handle);
        }

        if (time_tolerance_.nanoseconds()) {
          handle = bc_.addTransformableRequest(callback_handle_, target_frame, frame_id, tf2::timeFromSec(
                (stamp + time_tolerance_).seconds()));
          if (handle == 0xffffffffffffffffULL) {
            // never transformable
            messageDropped(evt, filter_failure_reasons::OutTheBack);
            return;
          } else if (handle == 0) {
            ++info.success_count;
          } else {
            info.handles.push_back(handle);
          }
        }
      }
    }


    // We can transform already
    if (info.success_count == expected_success_count_) {
      messageReady(evt);
    } else {
      // If this message is about to push us past our queue size, erase the oldest message
      if (queue_size_ != 0 && message_count_ + 1 > queue_size_) {

        // While we're using the reference keep a lock on the messages.
        std::unique_lock<std::mutex> unique_lock(messages_mutex_);

        ++dropped_message_count_;
        const MessageInfo & front = messages_.front();
        TF2_ROS_MESSAGEFILTER_DEBUG(
          "Removed oldest message because buffer is full, count now %d (frame_id=%s, stamp=%f)",
          message_count_,
          (mt::FrameId<M>::value(*front.event.getMessage())).c_str(),
          mt::TimeStamp<M>::value(*front.event.getMessage()).seconds());

        V_TransformableRequestHandle::const_iterator it = front.handles.begin();
        V_TransformableRequestHandle::const_iterator end = front.handles.end();
        for (; it != end; ++it) {
          bc_.cancelTransformableRequest(*it);
        }

        messageDropped(front.event, filter_failure_reasons::Unknown);

        messages_.pop_front();
        --message_count_;
      }

      // Add the message to our list
      info.event = evt;
      messages_.push_back(info);
      ++message_count_;
    }

    TF2_ROS_MESSAGEFILTER_DEBUG("Added message in frame %s at time %.3f, count now %d",
      frame_id.c_str(), stamp.seconds(), message_count_);
    ++incoming_message_count_;
  }

  /**
   * \brief Manually add a message into this filter.
   * \note If the message (or any other messages in the queue) are immediately transformable this will immediately call through to the output callback, possibly
   * multiple times
   */
  void add(const MConstPtr & message)
  {
    using builtin_interfaces::msg::Time;
    std::shared_ptr<std::map<std::string, std::string>> header(new std::map<std::string,
      std::string>);

    (*header)["callerid"] = "unknown";
    Time t = node_clock_->get_clock()->now();
    add(MEvent(message, header, t));
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
    message_count_ = 0;
    successful_transform_count_ = 0;
    failed_out_the_back_count_ = 0;
    transform_message_count_ = 0;
    incoming_message_count_ = 0;
    dropped_message_count_ = 0;
    time_tolerance_ = rclcpp::Duration(0, 0);
    warned_about_empty_frame_id_ = false;
    expected_success_count_ = 1;

    callback_handle_ = bc_.addTransformableCallback(std::bind(&MessageFilter::transformable,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5));
  }

  void transformable(
    tf2::TransformableRequestHandle request_handle, const std::string & target_frame,
    const std::string & source_frame,
    tf2::TimePoint time, tf2::TransformableResult result)
  {
    (void)target_frame;
    (void)source_frame;
    (void)time;
    namespace mt = message_filters::message_traits;

    // find the message this request is associated with
    typename L_MessageInfo::iterator msg_it = messages_.begin();
    typename L_MessageInfo::iterator msg_end = messages_.end();
    for (; msg_it != msg_end; ++msg_it) {
      MessageInfo & info = *msg_it;
      V_TransformableRequestHandle::const_iterator handle_it = std::find(
        info.handles.begin(), info.handles.end(), request_handle);
      if (handle_it != info.handles.end()) {
        // found msg_it
        ++info.success_count;
        break;
      }
    }

    if (msg_it == msg_end) {
      return;
    }

    const MessageInfo & info = *msg_it;
    if (info.success_count < expected_success_count_) {
      return;
    }

    bool can_transform = true;
    const MConstPtr & message = info.event.getMessage();
    std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
    rclcpp::Time stamp = mt::TimeStamp<M>::value(*message);

    if (result == tf2::TransformAvailable) {
      std::unique_lock<std::mutex> frames_lock(target_frames_mutex_);
      // make sure we can still perform all the necessary transforms
      typename V_string::iterator it = target_frames_.begin();
      typename V_string::iterator end = target_frames_.end();
      for (; it != end; ++it) {
        const std::string & target = *it;
        if (!bc_.canTransform(target, frame_id, tf2::timeFromSec(stamp.seconds()))) {
          can_transform = false;
          break;
        }

        if (time_tolerance_.nanoseconds()) {
          if (!bc_.canTransform(target, frame_id,
            tf2::timeFromSec((stamp + time_tolerance_).seconds())))
          {
            can_transform = false;
            break;
          }
        }
      }
    } else {
      can_transform = false;
    }

    // We will be mutating messages now, require unique lock
    std::unique_lock<std::mutex> lock(messages_mutex_);
    if (can_transform) {
      TF2_ROS_MESSAGEFILTER_DEBUG("Message ready in frame %s at time %.3f, count now %d",
        frame_id.c_str(), stamp.seconds(), message_count_ - 1);

      ++successful_transform_count_;
      messageReady(info.event);
    } else {
      ++dropped_message_count_;

      TF2_ROS_MESSAGEFILTER_DEBUG("Discarding message in frame %s at time %.3f, count now %d",
        frame_id.c_str(), stamp.seconds(), message_count_ - 1);
      messageDropped(info.event, filter_failure_reasons::Unknown);
    }

    messages_.erase(msg_it);
    --message_count_;
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
      if (incoming_message_count_ - message_count_ == 0) {
        return;
      }

      double dropped_pct = static_cast<double>(dropped_message_count_) /
        static_cast<double>(incoming_message_count_ - message_count_);
      if (dropped_pct > 0.95) {
        TF2_ROS_MESSAGEFILTER_WARN(
          "Dropped %.2f%% of messages so far. Please turn the [%s.message_notifier] rosconsole logger to DEBUG for more information.", dropped_pct * 100,
          "tf2_ros_message_filter");
        next_failure_warning_ = node_clock_->get_clock()->now() + rclcpp::Duration(60, 0);

        if (static_cast<double>(failed_out_the_back_count_) / static_cast<double>(dropped_message_count_) > 0.5) {
          TF2_ROS_MESSAGEFILTER_WARN(
            "  The majority of dropped messages were due to messages growing older than the TF cache time.  The last message's timestamp was: %f, and the last frame_id was: %s",
            last_out_the_back_stamp_.seconds(), last_out_the_back_frame_.c_str());
        }
      }
    }
  }

  // TODO(clalancette): reenable this once we have underlying support for callback queues
#if 0
  struct CBQueueCallback : public ros::CallbackInterface
  {
    CBQueueCallback(MessageFilter* filter, const MEvent& event, bool success, FilterFailureReason reason)
    : event_(event)
    , filter_(filter)
    , reason_(reason)
    , success_(success)
    {}


    virtual CallResult call()
    {
      if (success_)
      {
        filter_->signalMessage(event_);
      }
      else
      {
        filter_->signalFailure(event_, reason_);
      }

      return Success;
    }

  private:
    MEvent event_;
    MessageFilter* filter_;
    FilterFailureReason reason_;
    bool success_;
  };
#endif

  void messageDropped(const MEvent& evt, FilterFailureReason reason)
  {
    // TODO(clalancette): reenable this once we have underlying support for callback queues
#if 0
    if (callback_queue_) {
      ros::CallbackInterfacePtr cb(new CBQueueCallback(this, evt, false, reason));
      callback_queue_->addCallback(cb, (uint64_t)this);
    }
    else
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
      ros::CallbackInterfacePtr cb(new CBQueueCallback(this, evt, true, filter_failure_reasons::Unknown));
      callback_queue_->addCallback(cb, (uint64_t)this);
    }
    else
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
    RCLCPP_INFO(node_logging_->get_logger(), "[%s] Drop message: frame '%s' at time %.3f for reason(%d)",
      __func__, frame_id.c_str(), stamp.seconds(), reason);
  }

  static std::string stripSlash(const std::string & in)
  {
    if (!in.empty() && (in [0] == '/')) {
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
  tf2::BufferCore & bc_;
  ///< The frames we need to be able to transform to before a message is ready
  V_string target_frames_;
  std::string target_frames_string_;
  ///< A mutex to protect access to the target_frames_ list and target_frames_string.
  std::mutex target_frames_mutex_;
  ///< The maximum number of messages we queue up
  uint32_t queue_size_;
  tf2::TransformableCallbackHandle callback_handle_;

  typedef std::vector<tf2::TransformableRequestHandle> V_TransformableRequestHandle;
  struct MessageInfo
  {
    MessageInfo()
    : success_count(0) {}

    MEvent event;
    V_TransformableRequestHandle handles;
    uint64_t success_count;
  };
  typedef std::list<MessageInfo> L_MessageInfo;
  L_MessageInfo messages_;

  ///< The number of messages in the list.  Used because \<container\>.size() may have linear cost
  uint64_t message_count_;
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

  ///< Provide additional tolerance on time for messages which are stamped but can have associated duration
  rclcpp::Duration time_tolerance_ = rclcpp::Duration(0, 0);

  message_filters::Connection message_connection_;
  message_filters::Connection message_connection_failure;
};

} // namespace tf2

#endif
