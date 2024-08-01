#ifndef RM_SERVICE_GLOBAL_H
#define RM_SERVICE_GLOBAL_H

#ifdef __linux
#define RM_SERVICESHARED_EXPORT
#endif

#if _WIN32
#if defined(RM_SERVICE_LIBRARY)
#  define RM_SERVICESHARED_EXPORT __declspec(dllexport)
#else
#  define RM_SERVICESHARED_EXPORT __declspec(dllexport)
#endif
#endif

#endif // RM_SERVICE_GLOBAL_H
