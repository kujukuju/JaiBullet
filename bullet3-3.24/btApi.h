//
// Created by Kuju on 12/9/2022.
//

#pragma warning(disable: 4305)
#pragma warning(disable: 4267)
#pragma warning(disable: 4244)

#ifndef BT_API_H
#define BT_API_H

#ifdef BT_SHARED
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BT_API __attribute__ ((dllexport))
  #else
    #define BT_API __declspec(dllexport)
  #endif
  #else
    #if __GNUC__ >= 4
      #define BT_API __attribute__ ((visibility ("default")))
    #else
      #define BT_API
    #endif
  #endif
#else
#define BT_API
#endif

#endif
