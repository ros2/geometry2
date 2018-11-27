#ifndef TF2_FILTER__VISIBILITY_CONTROL_H_
#define TF2_FILTER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TF2_FILTER_EXPORT __attribute__ ((dllexport))
    #define TF2_FILTER_IMPORT __attribute__ ((dllimport))
  #else
    #define TF2_FILTER_EXPORT __declspec(dllexport)
    #define TF2_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TF2_FILTER_BUILDING_LIBRARY
    #define TF2_FILTER_PUBLIC TF2_FILTER_EXPORT
  #else
    #define TF2_FILTER_PUBLIC TF2_FILTER_IMPORT
  #endif
  #define TF2_FILTER_PUBLIC_TYPE TF2_FILTER_PUBLIC
  #define TF2_FILTER_LOCAL
#else
  #define TF2_FILTER_EXPORT __attribute__ ((visibility("default")))
  #define TF2_FILTER_IMPORT
  #if __GNUC__ >= 4
    #define TF2_FILTER_PUBLIC __attribute__ ((visibility("default")))
    #define TF2_FILTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TF2_FILTER_PUBLIC
    #define TF2_FILTER_LOCAL
  #endif
  #define TF2_FILTER_PUBLIC_TYPE
#endif

#endif  // TF2_FILTER__VISIBILITY_CONTROL_H_
