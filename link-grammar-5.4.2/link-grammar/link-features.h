#ifndef LINK_FEATURES_H
#define LINK_FEATURES_H

#if defined(_MSC_VER) && !defined(LINK_GRAMMAR_DLL_EXPORT)
#define LINK_GRAMMAR_DLL_EXPORT 1
#endif

#ifdef  __cplusplus
# define LINK_BEGIN_DECLS  extern "C" {
# define LINK_END_DECLS    }
#else
# define LINK_BEGIN_DECLS
# define LINK_END_DECLS
#endif

#ifndef link_public_api
# if defined(_MSC_VER) && !defined(LINK_GRAMMAR_STATIC)
#  if !defined LINK_GRAMMAR_DLL_EXPORT
#   error !defined LINK_GRAMMAR_DLL_EXPORT
#  endif
#  if LINK_GRAMMAR_DLL_EXPORT
#   define link_public_api(x) __declspec(dllexport) x
#  else
#   define link_public_api(x) __declspec(dllimport) x
#  endif
# else
#  define link_public_api(x) x
# endif
#endif

#define LINK_MAJOR_VERSION 5
#define LINK_MINOR_VERSION 4
#define LINK_MICRO_VERSION 2

#define LINK_VERSION_STRING "5.4.2"

#endif
