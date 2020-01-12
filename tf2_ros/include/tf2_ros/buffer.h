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
 */

/** \author Wim Meeussen */

#ifndef TF2_ROS_BUFFER_H
#define TF2_ROS_BUFFER_H

#include <memory>
#include <mutex>
#include <unordered_map>

#include <tf2_ros/async_buffer_interface.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/visibility_control.h>
#include <tf2/buffer_core.h>
#include <tf2_msgs/srv/frame_graph.hpp>
#include <rclcpp/rclcpp.hpp>
//TODO(tfoote)  review removal #include <tf2/convert.h>


namespace tf2_ros
{

  /** \brief Standard implementation of the tf2_ros::BufferInterface abstract data type.
   *
   * Inherits tf2_ros::BufferInterface and tf2::BufferCore.
   * Stores known frames and offers a ROS service, "tf_frames", which responds to client requests
   * with a response containing a tf2_msgs::FrameGraph representing the relationship of known frames.
   */
  class Buffer: public BufferInterface, public AsyncBufferInterface, public tf2::BufferCore
  {
  public:
    using tf2::BufferCore::lookupTransform;
    using tf2::BufferCore::canTransform;

    /**
     * @brief  Constructor for a Buffer object
     * @param clock A clock to use for time and sleeping
     * @param cache_time How long to keep a history of transforms
     * @param debug Whether to advertise the view_frames service that exposes debugging information from the buffer
     * @return 
     */
    TF2_ROS_PUBLIC Buffer(rclcpp::Clock::SharedPtr clock, tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));

    /** \brief Get the transform between two frames by frame ID.
     * \param target_frame The frame to which data should be transformed
     * \param source_frame The frame where the data originated
     * \param time The time at which the value of the transform is desired. (0 will get the latest)
     * \param timeout How long to block before failing
     * \return The transform between the frames
     *
     * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
     * tf2::ExtrapolationException, tf2::InvalidArgumentException
     */
    TF2_ROS_PUBLIC
    geometry_msgs::msg::TransformStamped
    lookupTransform(const std::string& target_frame, const std::string& source_frame,
                    const tf2::TimePoint& time, const tf2::Duration timeout) const override;

    /** \brief Get the transform between two frames by frame ID.
     * \sa lookupTransform(const std::string&, const std::string&, const tf2::TimePoint&,
                           const tf2::Duration)
     */
    TF2_ROS_PUBLIC
    geometry_msgs::msg::TransformStamped
    lookupTransform(const std::string& target_frame, const std::string& source_frame,
      const rclcpp::Time & time, const rclcpp::Duration timeout=rclcpp::Duration(0)) const
    {
      return lookupTransform(target_frame, source_frame, fromRclcpp(time), fromRclcpp(timeout));
    }

    /** \brief Get the transform between two frames by frame ID assuming fixed frame.
     * \param target_frame The frame to which data should be transformed
     * \param target_time The time to which the data should be transformed. (0 will get the latest)
     * \param source_frame The frame where the data originated
     * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
     * \param fixed_frame The frame in which to assume the transform is constant in time. 
     * \param timeout How long to block before failing
     * \return The transform between the frames
     *
     * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
     * tf2::ExtrapolationException, tf2::InvalidArgumentException
     */
    TF2_ROS_PUBLIC
    geometry_msgs::msg::TransformStamped
    lookupTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
                    const std::string& source_frame, const tf2::TimePoint& source_time,
                    const std::string& fixed_frame, const tf2::Duration timeout) const override;

    /** \brief Get the transform between two frames by frame ID assuming fixed frame.
     * \sa lookupTransform(const std::string&, const tf2::TimePoint&,
                           const std::string&, const tf2::TimePoint&,
                           const std::string&, const tf2::Duration)
     */
    TF2_ROS_PUBLIC
    geometry_msgs::msg::TransformStamped
    lookupTransform(const std::string & target_frame, const rclcpp::Time & target_time,
      const std::string & source_frame, const rclcpp::Time & source_time,
      const std::string & fixed_frame, const rclcpp::Duration timeout=rclcpp::Duration(0)) const
    {
      return lookupTransform(
        target_frame, fromRclcpp(target_time),
        source_frame, fromRclcpp(source_time),
        fixed_frame, fromRclcpp(timeout));
    }

    /** \brief Test if a transform is possible
     * \param target_frame The frame into which to transform
     * \param source_frame The frame from which to transform
     * \param target_time The time at which to transform
     * \param timeout How long to block before failing
     * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
     * \return True if the transform is possible, false otherwise 
     */
    TF2_ROS_PUBLIC
    bool
    canTransform(const std::string& target_frame, const std::string& source_frame, 
                 const tf2::TimePoint& target_time, const tf2::Duration timeout, std::string* errstr = NULL) const override;

    /** \brief Test if a transform is possible
     * \sa canTransform(const std::string&, const std::string&,
                        const tf2::TimePoint&, const tf2::Duration, std::string*)
     */
    TF2_ROS_PUBLIC
    bool
    canTransform(const std::string & target_frame, const std::string & source_frame,
      const rclcpp::Time & time, const rclcpp::Duration timeout=rclcpp::Duration(0),
      std::string * errstr=NULL) const
    {
      return canTransform(target_frame, source_frame, fromRclcpp(time), fromRclcpp(timeout), errstr);
    }

    /** \brief Test if a transform is possible
     * \param target_frame The frame into which to transform
     * \param target_time The time into which to transform
     * \param source_frame The frame from which to transform
     * \param source_time The time from which to transform
     * \param fixed_frame The frame in which to treat the transform as constant in time
     * \param timeout How long to block before failing
     * \param errstr A pointer to a string which will be filled with why the transform failed, if not NULL
     * \return True if the transform is possible, false otherwise 
     */
    TF2_ROS_PUBLIC
    bool
      canTransform(const std::string& target_frame, const tf2::TimePoint& target_time,
                   const std::string& source_frame, const tf2::TimePoint& source_time,
                   const std::string& fixed_frame, const tf2::Duration timeout, std::string* errstr = NULL) const override;

    /** \brief Test if a transform is possible
     * \sa
      canTransform(const std::string&, const tf2::TimePoint&,
                   const std::string&, const tf2::TimePoint&,
                   const std::string&, const tf2::Duration, std::string*)
     */
    TF2_ROS_PUBLIC
    bool
    canTransform(const std::string & target_frame, const rclcpp::Time & target_time,
      const std::string & source_frame, const rclcpp::Time & source_time,
      const std::string & fixed_frame, const rclcpp::Duration timeout=rclcpp::Duration(0),
      std::string * errstr=NULL) const
    {
      return canTransform(
        target_frame, fromRclcpp(target_time),
        source_frame, fromRclcpp(source_time),
        fixed_frame, fromRclcpp(timeout),
        errstr);
    }

   /** \brief Wait for a transform between two frames to become available.
    *
    * Before this method can be called, a tf2_ros::CreateTimerInterface must be registered
    * by first calling setCreateTimerInterface.
    * If no tf2_ros::CreateTimerInterface is set, then a tf2_ros::CreateTimerInterfaceException
    * is thrown.
    *
    * \param target_frame The frame into which to transform.
    * \param source_frame The frame from which to tranform.
    * \param time The time at which to transform.
    * \param timeout Duration after which waiting will be stopped.
    * \param callback The function to be called when the transform becomes available or a timeout
    *   occurs. In the case of timeout, an exception will be set on the future.
    * \return A future to the requested transform. If a timeout occurs a `tf2::LookupException`
    *   will be set on the future.
    */
    TF2_ROS_PUBLIC
    TransformStampedFuture
    waitForTransform(const std::string& target_frame, const std::string& source_frame, const tf2::TimePoint& time,
                     const tf2::Duration& timeout, TransformReadyCallback callback) override;

   /** \brief Wait for a transform between two frames to become available.
    * \sa waitForTransform(const std::string &, const std::string &, const tf2::TimePoint &,
                           const tf2::Duration &, TransformReadyCallback);
    */
    TF2_ROS_PUBLIC
    TransformStampedFuture
    waitForTransform(const std::string & target_frame, const std::string & source_frame,
                     const rclcpp::Time & time,
                     const rclcpp::Duration & timeout, TransformReadyCallback callback)
    {
      return waitForTransform(
        target_frame, source_frame,
        fromRclcpp(time), fromRclcpp(timeout),
        callback);
    }

    TF2_ROS_PUBLIC
    inline void
    setCreateTimerInterface(CreateTimerInterface::SharedPtr create_timer_interface)
    {
      timer_interface_ = create_timer_interface;
    }
    
  private:
    void timerCallback(const TimerHandle & timer_handle,
                       std::shared_ptr<std::promise<geometry_msgs::msg::TransformStamped>> promise,
                       TransformStampedFuture future,
                       TransformReadyCallback callback);

    bool getFrames(tf2_msgs::srv::FrameGraph::Request& req, tf2_msgs::srv::FrameGraph::Response& res) ;

    void onTimeJump(const rcl_time_jump_t & jump);

    // conditionally error if dedicated_thread unset.
    bool checkAndErrorDedicatedThreadPresent(std::string* errstr) const;

//TODO(tfoote)renable framegraph service
//    ros::ServiceServer frames_server_;

    /// \brief A clock to use for time and sleeping
    rclcpp::Clock::SharedPtr clock_;

    /// \brief Interface for creating timers
    CreateTimerInterface::SharedPtr timer_interface_;

    /// \brief A map from active timers to BufferCore request handles
    std::unordered_map<TimerHandle, tf2::TransformableRequestHandle> timer_to_request_map_;

    /// \brief A mutex on the timer_to_request_map_ data
    std::mutex timer_to_request_map_mutex_;

    /// \brief Reference to a jump handler registered to the clock
    rclcpp::JumpHandler::SharedPtr jump_handler_;
  }; // class 

static const std::string threading_error = "Do not call canTransform or lookupTransform with a timeout unless you are using another thread for populating data. Without a dedicated thread it will always timeout.  If you have a seperate thread servicing tf messages, call setUsingDedicatedThread(true) on your Buffer instance.";

  
} // namespace

#endif // TF2_ROS_BUFFER_H
