/**
* Copyright 2019, ftdlyc <yclu.cn@gmail.com>
* Licensed under the MIT license.
*/

#pragma once
#ifndef CALIB_CALIB_DEF_H
#define CALIB_CALIB_DEF_H

#if defined(_MSC_VER)

#define CALIB_INLINE __forceinline
#define CALIB_NO_INLINE

#if defined(CALIB_DLL_EXPORT)
#define CALIB_DLL_DECL __declspec(dllexport)
#else
#define CALIB_DLL_DECL __declspec(dllimport)
#endif
#define CALIB_DLL_LOCAL

#elif defined(__GNUC__)

#define CALIB_INLINE inline __attribute__((always_inline))
#define CALIB_NO_INLINE __attribute__((noinline))

#if defined(CALIB_DLL_EXPORT)
#define CALIB_DLL_DECL __attribute__((visibility("default")))
#define CALIB_DLL_LOCAL __attribute__((visibility("hidden")))
#else
#define CALIB_DLL_DECL
#define CALIB_DLL_LOCAL __attribute__((visibility("hidden")))
#endif

#else
#error Unsupported compiler.
#endif

#endif //CALIB_CALIB_DEF_H
