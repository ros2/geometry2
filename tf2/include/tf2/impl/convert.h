// Copyright 2013, Open Source Robotics Foundation, Inc. All rights reserved.
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
//    * Neither the name of the Open Source Robotics Foundation nor the names of its
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

/** \author Vincent Rabaud, Bjarne von Horn */

#ifndef TF2__IMPL__CONVERT_H_
#define TF2__IMPL__CONVERT_H_

#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>

#include "../time.h"
#include "forward.h"

#include <type_traits>


namespace tf2
{
namespace impl
{

/// Helper to always trigger \c static_assert s
template<typename T>
constexpr bool alwaysFalse = false;

/** \brief Check whether a message is stamped and has a std_msgs::msg::Header member.
 *
 * It will be indicated with the static member valiable \c value .
 * \tparam Message The message to check
 */
template<typename Message, typename = int>
struct MessageHasStdHeader : std::false_type {};

/** \brief Check whether a message is stamped and has a std_msgs::msg::Header member.
 *
 * It will be indicated with the static member valiable \c value .
 * \tparam Message The message to check
 */
template<typename Message>
class MessageHasStdHeader<Message, decltype(&Message::header, 0)>
{
  template<typename Alloc>
  static std::true_type headerIsStdHeader(std_msgs::msg::Header_<Alloc>);
  template<typename = void>
  static std::false_type headerIsStdHeader(...);

public:
  /// true if Message has a member \c header of type std_msgs::msg::Header.
  static constexpr bool value = decltype(headerIsStdHeader(std::declval<Message>().header))::value;
};

/**
 * \brief Mapping between Datatypes (like \c Vector3d ) and their default ROS Message types.
 *
 * This struct should be specialized for each non-Message datatypes,
 * and it should contain an alias of the Message class with the name \c type .
 * This alias will be used to deduce the return value of tf2::toMsg().
 *
 * \tparam Datatype Non-Message datatype like \c Vector3d .
 */
template<class Datatype, class>
struct DefaultMessageForDatatype
{
  static_assert(
    alwaysFalse<Datatype>, "No default message type defined, "
    "please check your header file includes!");
  // using type = ...;
};

/**
 * \brief Transform type for the default implementation of tf2::doTransform().
 *
 * The default implementation of tf2::doTransform() needs to convert the
 * TransformStamped parameter into a type to perform the transform operation
 * \code
 * tf2::convert(transform, converted_transform);
 * out = converted_transform * in;
 * \endcode
 * So, this struct needs to be specialized for each type which is passed
 * to the default implementation of tf2::doTransform().
 * It needs to contain an alias of the transform datatype named \c type .
 *
 * \tparam Datatype Datatype to be transformed.
 */
template<class Datatype, class>
struct DefaultTransformType
{
  static_assert(
    alwaysFalse<Datatype>, "No default transform type defined, "
    "please check your header file includes!");
  // using type = ...;
};

/**
 * \brief Conversion details between a Message and a non-Message datatype.
 * \tparam Datatype Non-Message datatype like \c Vector3d
 * \tparam Message  The ROS Message class
 *
 * The specializations of this struct should contain two static methods,
 * which convert a ROS Message into the requested datatype and vice versa.
 * They should have the following signature:
 * \code
 * template<>
 * struct defautMessage<Datatype, Message>
 * {
 *   static void toMsg(const Datatype&, Message&);
 *   static void fromMsg(const Message&, Datatype&);
 * }:
 * \endcode
 * Note that the conversion between ( tf2::Stamped\<Datatype\> and
 * geometry_msgs::...Stamped ) and
 * ( tf2::WithCovarianceStamped\<Datatype\> and geometry_msgs::...WithCovarianceStamped )
 * is done automatically.
 */
template<class Datatype, class Message, class>
struct ConversionImplementation
{
  static_assert(
    alwaysFalse<Datatype>, "No Conversion Implementation available, "
    "please check your header file includes!");
  // void toMsg(const Datatype&, Message&);
  // void fromMsg(const Message&, Datatype&);
};

/**
 * \brief Mapping of unstamped Messages for stamped Messages
 *
 * This struct contains utility methods to access the data member of a stamped ROS message
 * and an alias (named \c UntampedType ) of the unstamped message type.
 * It is needed for the conversion of stamped datatypes,
 * so that only the conversions of unstamped datatypes has do be implemented.
 * For example, a \c geometry_msgs::Vector3Stamped has two members,
 * the \c header (which contains a timestamp and a frame ID) and the \c vector itself.
 * For this class, the specialization should look like
 * \code
 * template<>
 * struct StampedMessageTraits<geometry_msgs::Vector3Stamped>
 * {
 *  using UntampedType = geometry_msgs::Vector3;
 *  static geometry_msgs::Vector3& accessMessage(geometry_msgs::Vector3Stamped& vs)
 *  {
 *     return vs.vector;
 *  }
 *  static geometry_msgs::Vector3 getMessage(const geometry_msgs::Vector3Stamped& vs)
 *  {
 *     return vs.vector;
 *  }
 * };
 * \endcode
 * The both almost identical methods are required to keep const-correctness.
 *
 * If the message is a stamped message with a covariance member,
 * accessMessage() should return the underlying message (e.g. Pose)
 * and accessCovariance() should return the covariance accordingly.
 *
 * \tparam StampedMessage The datatype of the ros message
 */
template<class StampedMessage>
struct StampedMessageTraits
{
  static_assert(
    alwaysFalse<StampedMessage>, "No traits for this stamped message type available, "
    "please check your header file includes!");
  // using UntampedType = ...;
  // static UntampedType& accessMessage(StampedMsg &);
  // static UntampedType getMessage(StampedMsg const&);
  // static CovarianceType & accessCovariance(MsgWithCovarianceStamped &);
  // static CovarianceType getCovariance(MsgWithCovarianceStamped const&);
};

/**
 * \brief Mapping of stamped Messages for unstamped Messages
 *
 * This struct is needed for the deduction of the return type of
 * tf2::convert() for tf2::Stamped\<\> datatypes.
 * Its specializations should contain an alias (named \c StampedType )
 * of the stamped type.
 * Example:
 * \code
 * template<>
 * struct UnstampedMessageTraits<geometry_msgs::Vector3>
 * {
 *    using StampedType = geometry_msgs::Vector3Stamped;
 * };
 * \endcode
 *
 * If messages with covariance are also available,
 * an alias with the name \c StampedTypeWithCovariance
 * to the accoring message type should be declared.
 *
 * \tparam UnstampedMessage Type of the ROS message which is not stamped
 */
template<class UnstampedMessage>
struct UnstampedMessageTraits
{
  static_assert(
    alwaysFalse<UnstampedMessage>, "No traits for this message type available, "
    "please check your header file includes!");
  // using StampedType = ...;
  // using StampedTypeWithCovariance = ...;
};

/**
 * \brief Partial specialization of impl::DefaultMessageForDatatype for stamped types.
 *
 * The deduction of the default ROS message type of a tf2::Stamped\<T\> type is
 * based on the default ROS message type of \c T .
 * \tparam T The unstamped datatype (not a ROS message)
 */
template<class T>
struct DefaultMessageForDatatype<tf2::Stamped<T>>
{
  using type =
    typename UnstampedMessageTraits<typename DefaultMessageForDatatype<T>::type>::StampedType;
};

/**
 * \brief Partial specialization of impl::DefaultMessageForDatatype for stamped types with covariance.
 *
 * The deduction of the default ROS message type of a tf2::WithCovarianceStamped\<T\> type is
 * based on the default ROS message type of \c T .
 * \tparam T The unstamped datatype (not a ROS message)
 */
template<class T>
struct DefaultMessageForDatatype<tf2::WithCovarianceStamped<T>>
{
  using type =
    typename UnstampedMessageTraits<typename DefaultMessageForDatatype<T>::type>::
    StampedTypeWithCovariance;
};

/**
 * \brief Partial specialization of impl::ConversionImplementation for stamped types.
 *
 * This partial specialization provides the conversion implementation ( \c toMsg() and \c fromMsg() )
 * between stamped types ( non-message types of tf2::Stamped\<T\> and ROS message datatypes with a \c header member).
 * The timestamp and the frame ID are preserved during the conversion.
 * The implementation of tf2::toMsg() and tf2::fromMsg() for the unstamped types are required,
 * as well as a specialization of StampedMessageTraits.
 * \tparam Datatype Unstamped non-message type
 * \tparam StampedMessage Stamped ROS message type
 */
template<class Datatype, class StampedMessage>
struct ConversionImplementation<tf2::Stamped<Datatype>, StampedMessage>
{
  /// Typedefs and utility functions for the given message
  using traits = StampedMessageTraits<StampedMessage>;

  /** \brief Convert a stamped datatype to a stamped message.
   * \param[in] s Stamped datatype to covert.
   * \param[out] msg The stamped datatype, as a stamped message.
   */
  static void toMsg(const tf2::Stamped<Datatype> & s, StampedMessage & msg)
  {
    tf2::toMsg<>(static_cast<const Datatype &>(s), traits::accessMessage(msg));
    tf2::toMsg<>(s.stamp_, msg.header.stamp);
    msg.header.frame_id = s.frame_id_;
  }

  /** \brief Convert a stamped message to a stamped datatype.
   * \param[in] msg Stamped message to covert
   * \param[out] s The stamped message, as a stamped datatype.
   */
  static void fromMsg(const StampedMessage & msg, tf2::Stamped<Datatype> & s)
  {
    tf2::fromMsg<>(traits::getMessage(msg), static_cast<Datatype &>(s));
    tf2::fromMsg<>(msg.header.stamp, s.stamp_);
    s.frame_id_ = msg.header.frame_id;
  }
};

/**
 * \brief Partial specialization of impl::ConversionImplementation for stamped types with covariance.
 *
 * This partial specialization provides the conversion implementation ( \c toMsg() and \c fromMsg() )
 * between stamped types with covariance ( non-message types of tf2::WithCovarianceStamped\<T\>
 * and ROS message datatypes with a \c header member).
 * The covariance, the timestamp and the frame ID are preserved during the conversion.
 * The implementation of tf2::toMsg() and tf2::fromMsg() for the unstamped types without covariance
 * are required, as well as a specialization of StampedMessageTraits.
 * \tparam Datatype Unstamped non-message type
 * \tparam CovarianceStampedMessage Stamped ROS message type with covariance
 */
template<class Datatype, class CovarianceStampedMessage>
struct ConversionImplementation<tf2::WithCovarianceStamped<Datatype>, CovarianceStampedMessage>
{
  /// Typedefs and utility functions for the given message
  using traits = StampedMessageTraits<CovarianceStampedMessage>;

  /** \brief Convert a stamped datatype to a stamped message.
  * \param[in] in Stamped datatype to covert.
  * \param[out] out The stamped datatype, as a stamped message.
  */
  static void toMsg(tf2::WithCovarianceStamped<Datatype> const & in, CovarianceStampedMessage & out)
  {
    CovarianceStampedMessage tmp;
    tf2::toMsg<>(in.stamp_, tmp.header.stamp);
    tmp.header.frame_id = in.frame_id_;
    traits::accessCovariance(tmp) = tf2::covarianceNestedToRowMajor(in.cov_mat_);
    tf2::toMsg<>(static_cast<Datatype const &>(in), traits::accessMessage(tmp));
    out = std::move(tmp);
  }

  /** \brief Convert a stamped message to a stamped datatype.
   * \param[in] in Stamped message to covert
   * \param[out] out The stamped message, as a stamped datatype.
   */
  static void fromMsg(
    CovarianceStampedMessage const & in, tf2::WithCovarianceStamped<Datatype> & out)
  {
    tf2::WithCovarianceStamped<Datatype> tmp;
    tf2::fromMsg<>(in.header.stamp, tmp.stamp_);
    tf2::fromMsg<>(traits::getMessage(in), static_cast<Datatype &>(tmp));
    tmp.frame_id_ = in.header.frame_id;
    tmp.cov_mat_ = tf2::covarianceRowMajorToNested(traits::getCovariance(in));
    out = std::move(tmp);
  }
};

/** \brief Helper for tf2::convert().
 * \tparam IS_MESSAGE_A True if first argument is a message type.
 * \tparam IS_MESSAGE_B True if second argument is a message type.
 */
template<bool IS_MESSAGE_A, bool IS_MESSAGE_B>
class Converter
{
public:
  /** \brief Implementation of tf2::convert() depending of the argument types.
   * \param[in] a Source of conversion.
   * \param[out] b Target of conversion.
   * \tparam A Type of first argument.
   * \tparam B Type of second argument.
   */
  template<typename A, typename B>
  static void convert(const A & a, B & b);

  // The case where both A and B are messages should not happen: if you have two
  // messages that are interchangeable, well, that's against the ROS purpose:
  // only use one type. Worst comes to worst, specialize the original convert
  // function for your types.
  // if B == A, the templated version of convert with only one argument will be
  // used.
  //
  static_assert(
    !(IS_MESSAGE_A && IS_MESSAGE_B),
    "Conversion between two Message types is not supported!");
};

/** \brief Implementation of tf2::convert() for message-to-datatype conversion.
 * \param[in] a Message to convert.
 * \param[out] b Datatype to convert to.
 * \tparam A Message type.
 * \tparam B Datatype.
 */
template<>
template<typename A, typename B>
inline void Converter<true, false>::convert(const A & a, B & b)
{
  fromMsg<>(a, b);
}

/** \brief Implementation of tf2::convert() for datatype-to-message converiosn.
 * \param[in] a Datatype to convert.
 * \param[out] b Message to convert to.
 * \tparam A Datatype.
 * \tparam B Message.
 */
template<>
template<typename A, typename B>
inline void Converter<false, true>::convert(const A & a, B & b)
{
  b = toMsg<A, B>(a);
}

/** \brief Implementation of tf2::convert() for datatypes.
 * Converts the first argument to a message
 * (usually \c impl::DefaultMessageForDatatype<A>::type )
 * and then converts the message to the second argument.
 * \param[in] a Source of conversion.
 * \param[out] b Target of conversion.
 * \tparam A Datatype of first argument.
 * \tparam B Datatype of second argument.
 */
template<>
template<typename A, typename B>
inline void Converter<false, false>::convert(const A & a, B & b)
{
  fromMsg<>(toMsg<>(a), b);
}

/**
 * \brief Default implementation for extracting timestamps and frame IDs.
 *
 * Both static member functions are for stamped ROS messages.
 *
 * \tparam T Arbitrary datatype
 */
template<typename T, int>
struct StampedAttributesHelper
{
  static_assert(
    MessageHasStdHeader<T>::value,
    "Trying to use default implementation for stamped message, "
    "but the datatype does not have a Header member.");

  /**\brief Get the timestamp from data
   * \param[in] t The data input.
   * \return The timestamp associated with the data.
   */
  static tf2::TimePoint getTimestamp(const T & t)
  {
    tf2::TimePoint timestamp;
    tf2::fromMsg<>(t.header.stamp, timestamp);
    return timestamp;
  }
  /**\brief Get the frame_id from data
   * \param[in] t The data input.
   * \return The frame_id associated with the data.
   */
  static std::string getFrameId(const T & t)
  {
    return t.header.frame_id;
  }
};

/**
 * \brief Partial specialization of StampedAttributesHelper for tf2::Stamped\<\> types
 */
template<typename T>
struct StampedAttributesHelper<tf2::Stamped<T>>
{
  /** \brief Get the timestamp from data
   * \param t The data input.
   * \return The timestamp associated with the data.
   */
  static tf2::TimePoint getTimestamp(const tf2::Stamped<T> & t) {return t.stamp_;}
  /** \brief Get the frame_id from data
   * \param t The data input.
   * \return The frame_id associated with the data.
   */
  static std::string getFrameId(const tf2::Stamped<T> & t) {return t.frame_id_;}
};

/**
 * \brief Partial specialization of StampedAttributesHelper for tf2::WithCovarianceStamped\<\> types
 */
template<typename T>
struct StampedAttributesHelper<tf2::WithCovarianceStamped<T>>
{
  /** \brief Get the timestamp from data
   * \param t The data input.
   * \return The timestamp associated with the data.
   */
  static tf2::TimePoint getTimestamp(const tf2::WithCovarianceStamped<T> & t) {return t.stamp_;}
  /** \brief Get the frame_id from data
   * \param t The data input.
   * \return The frame_id associated with the data.
   */
  static std::string getFrameId(const tf2::WithCovarianceStamped<T> & t) {return t.frame_id_;}
};

/** \brief Generic conversion of a vector and a vector-like message.
 *
 * \tparam VectorType Datatype of the Vector.
 * \tparam Message Message type, like geometry_msgs::msg::Vector3.
 */
template<class VectorType, class Message>
struct DefaultVectorConversionImplementation
{
  /** \brief Convert a vector type to a vector-like message.
   * \param[in] in The vector to convert.
   * \param[out] msg The converted vector, as a message.
   */
  static void toMsg(const VectorType & in, Message & msg)
  {
    msg.x = in[0];
    msg.y = in[1];
    msg.z = in[2];
  }

  /** \brief Convert a vector-like message type to a vector type.
   * \param[in] msg The message to convert.
   * \param[out] out The message converted to a vector type.
   */
  static void fromMsg(const Message & msg, VectorType & out)
  {
    out = VectorType(msg.x, msg.y, msg.z);
  }
};

/** \brief Generic conversion of a quaternion and
 * a geometry_msgs::msg::Quaternion message.
 *
 * \tparam QuaternionType Datatype of the Vector.
 */
template<class QuaternionType>
struct DefaultQuaternionConversionImplementation
{
  /** \brief Convert a quaternion type to a Quaternion message.
   * \param[in] in The quaternion convert.
   * \param[out] msg The quaternion converted to a Quaternion message.
   */
  static void toMsg(const QuaternionType & in, geometry_msgs::msg::Quaternion & msg)
  {
    msg.x = in[0];
    msg.y = in[1];
    msg.z = in[2];
    msg.w = in[3];
  }

  /** \brief Convert a Quaternion message type to a quaternion type.
   * \param[in] msg The Quaternion message to convert.
   * \param[out] out The Quaternion message converted to a quaternion type.
   */
  static void fromMsg(const geometry_msgs::msg::Quaternion & msg, QuaternionType & out)
  {
    out = QuaternionType(msg.x, msg.y, msg.z, msg.w);
  }
};

}  // namespace impl
}  // namespace tf2

#endif  // TF2__IMPL__CONVERT_H_
