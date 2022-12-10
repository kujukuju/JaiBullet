//
// Created by Kuju on 12/9/2022.
//

#ifndef BT_API_H
#define BT_API_H

#ifdef BT_SHARED
#if defined _WIN32 || defined __CYGWIN__
    #ifdef box2d_EXPORTS
      #ifdef __GNUC__
        #define BT_API __attribute__ ((dllexport))
      #else
        #define BT_API __declspec(dllexport)
      #endif
    #else
      #ifdef __GNUC__
        #define BT_API __attribute__ ((dllimport))
      #else
        #define BT_API __declspec(dllimport)
      #endif
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