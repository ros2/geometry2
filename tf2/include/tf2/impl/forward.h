// Copyright 2021, Bjarne von Horn. All rights reserved.
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

/** \author Bjarne von Horn */

#ifndef TF2__IMPL__FORWARD_H_
#define TF2__IMPL__FORWARD_H_

namespace tf2
{
namespace impl
{
template <class Datatype, class = void>
struct defaultMessage;
template <class Datatype, class = void>
struct DefaultTransformType;
}  // namespace impl

template <typename T>
class Stamped;

template <typename T>
class WithCovarianceStamped;

template <typename A, typename B>
B & toMsg(const A & a, B & b);

template <typename A, typename B = typename impl::defaultMessage<A>::type>
B toMsg(const A & a);

template <typename A, typename B>
void fromMsg(const A & a, B & b);

namespace impl
{
template <class Datatype, class Message, class = void>
struct ImplDetails;

template <class StampedMessage>
struct stampedMessageTraits;

template <class UnstampedMessage>
struct unstampedMessageTraits;

template <typename T, int = 0>
struct DefaultStampedImpl;

template <bool IS_MESSAGE_A, bool IS_MESSAGE_B>
class Converter;

}  // namespace impl
}  // namespace tf2

#endif  // TF2__IMPL__FORWARD_H_
