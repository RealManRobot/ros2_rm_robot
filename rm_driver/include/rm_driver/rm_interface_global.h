#ifndef RM_INTERFACE_GLOBAL_H
#define RM_INTERFACE_GLOBAL_H

#ifdef __linux
#define RM_INTERFACE_EXPORT
#endif

#if _WIN32
#if defined(RM_INTERFACE_LIBRARY)
#  define RM_INTERFACE_EXPORT __declspec(dllexport)
#else
#  define RM_INTERFACE_EXPORT __declspec(dllexport)
#endif
#endif

#endif // RM_INTERFACE_GLOBAL_H
