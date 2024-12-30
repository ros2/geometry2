/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef TF2__LINEARMATH__VECTOR3_H_
#define TF2__LINEARMATH__VECTOR3_H_

#ifdef __GNUC__
#define TF2_LINEARMATH_VECTOR3_H_PRAGMA(X) _Pragma(#X)
#define TF2_LINEARMATH_VECTOR3_H_DEPRECATED(MSG) TF2_LINEARMATH_VECTOR3_H_PRAGMA(GCC warning MSG)
#elif defined(_MSC_VER)
#define TF2_LINEARMATH_VECTOR3_H_STRINGIZE_(MSG) #MSG
#define TF2_LINEARMATH_VECTOR3_H_STRINGIZE(MSG) TF2_LINEARMATH_VECTOR3_H_STRINGIZE_(MSG)
#define TF2_LINEARMATH_VECTOR3_H_DEPRECATED(MSG) \
    __pragma(message(__FILE__ "(" TF2_LINEARMATH_VECTOR3_H_STRINGIZE(__LINE__) ") : Warning: " MSG))
#else
#define TF2_LINEARMATH_VECTOR3_H_DEPRECATED(MSG)
#endif

TF2_LINEARMATH_VECTOR3_H_DEPRECATED(  // NOLINT
  "This header is obsolete, please include 'tf2/LinearMath/Vector3.hpp' instead")

#include <tf2/LinearMath/Vector3.hpp>

#endif  // TF2__LINEARMATH__VECTOR3_H_
