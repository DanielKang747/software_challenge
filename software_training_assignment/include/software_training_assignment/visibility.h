#ifndef SOFTWARE_TRAINING_ASSIGNMENT__VISIBILITY_H_
#define SOFTWARE_TRAINING_ASSIGNMENT__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SOFTWARE_TRAINING_ASSIGNMENT_EXPORT __attribute__ ((dllexport))
    #define SOFTWARE_TRAINING_ASSIGNMENT_IMPORT __attribute__ ((dllimport))
  #else
    #define SOFTWARE_TRAINING_ASSIGNMENT_EXPORT __declspec(dllexport)
    #define SOFTWARE_TRAINING_ASSIGNMENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef SOFTWARE_TRAINING_BUILDING_DLL
    #define SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC SOFTWARE_TRAINING_ASSIGNMENT_EXPORT
  #else
    #define SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC SOFTWARE_TRAINING_ASSIGNMENT_IMPORT
  #endif
  #define SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC_TYPE SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
  #define SOFTWARE_TRAINING_ASSIGNMENT_LOCAL
#else
  #define SOFTWARE_TRAINING_ASSIGNMENT_EXPORT __attribute__ ((visibility("default")))
  #define SOFTWARE_TRAINING_ASSIGNMENT_IMPORT
  #if __GNUC__ >= 4
    #define SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC __attribute__ ((visibility("default")))
    #define SOFTWARE_TRAINING_ASSIGNMENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC
    #define SOFTWARE_TRAINING_ASSIGNMENT_LOCAL
  #endif
  #define SOFTWARE_TRAINING_ASSIGNMENT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SOFTWARE_TRAINING_ASSIGNMENT__VISIBILITY_H_
