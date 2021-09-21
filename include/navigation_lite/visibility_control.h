#ifndef NAVIGATION_LITE__VISIBILITY_CONTROL_H_
#define NAVIGATION_LITE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAVIGATION_LITE_EXPORT __attribute__ ((dllexport))
    #define NAVIGATION_LITE_IMPORT __attribute__ ((dllimport))
  #else
    #define NAVIGATION_LITE_EXPORT __declspec(dllexport)
    #define NAVIGATION_LITE_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAVIGATION_LITE_BUILDING_DLL
    #define NAVIGATION_LITE_PUBLIC NAVIGATION_LITE_EXPORT
  #else
    #define NAVIGATION_LITE_PUBLIC NAVIGATION_LITE_IMPORT
  #endif
  #define NAVIGATION_LITE_PUBLIC_TYPE NAVIGATION_LITE_PUBLIC
  #define NAVIGATION_LITE_LOCAL
#else
  #define NAVIGATION_LITE_EXPORT __attribute__ ((visibility("default")))
  #define NAVIGATION_LITE_IMPORT
  #if __GNUC__ >= 4
    #define NAVIGATION_LITE_PUBLIC __attribute__ ((visibility("default")))
    #define NAVIGATION_LITE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAVIGATION_LITE_PUBLIC
    #define NAVIGATION_LITE_LOCAL
  #endif
  #define NAVIGATION_LITE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_LITE__VISIBILITY_CONTROL_H_
