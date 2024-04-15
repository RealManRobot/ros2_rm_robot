#ifndef RM_BASE_GLOBAL_H
#define RM_BASE_GLOBAL_H

#ifdef __linux
#define RM_BASESHARED_EXPORT
#endif
#if _WIN32
#if defined(RM_BASE_LIBRARY)
#  define RM_BASESHARED_EXPORT __declspec(dllexport)
#else
#  define RM_BASESHARED_EXPORT __declspec(dllexport)
#endif
#endif
#endif // RM_BASE_GLOBAL_H
