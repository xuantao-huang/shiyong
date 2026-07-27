#pragma once

#ifdef LUCY_LOGGING
# include <oal/logfile/baselogger.hpp>
# define PREFIX "tracking"
# define LOG_TAG "svo"
# include <oal/logfile/enable_logging_macros.hpp>
#endif

#ifdef SVO_USE_ROS
# include <ros/console.h>
# define SVO_DEBUG_STREAM(x) ROS_DEBUG_STREAM(x)
# define SVO_INFO_STREAM(x) ROS_INFO_STREAM(x)
# define SVO_WARN_STREAM(x) ROS_WARN_STREAM(x)
# define SVO_WARN_STREAM_THROTTLE(rate, x) ROS_WARN_STREAM_THROTTLE(rate, x)
# define SVO_ERROR_STREAM(x) ROS_ERROR_STREAM(x)
#else
# ifdef LUCY_LOGGING
#   define SVO_INFO_STREAM(x) {std::stringstream ss; ss << x; LOGI("%s", ss.str().c_str()); }
#   define SVO_WARN_STREAM(x) {std::stringstream ss; ss << x; LOGW("%s", ss.str().c_str()); }
#   define SVO_DEBUG_STREAM(x) {std::stringstream ss; ss << x; LOGD("%s", ss.str().c_str()); }
#   define SVO_ERROR_STREAM(x) {std::stringstream ss; ss << x; LOGE("%s", ss.str().c_str()); }
# else
#   define SVO_INFO_STREAM(x) {std::cerr<<"\033[0;0m[INFO] VINS: "<<x<<"\033[0;0m"<<std::endl; }
#   define SVO_DEBUG_STREAM(x) {std::cerr<<"\033[0;0m[DEBUG] VINS: "<<x<<"\033[0;0m"<<std::endl; }
#   define SVO_WARN_STREAM(x) {std::cerr<<"\033[0;33m[WARN] VINS: "<<x<<"\033[0;0m"<<std::endl; }
#   define SVO_ERROR_STREAM(x) {std::cerr<<"\033[1;31m[ERROR] VINS: "<<x<<"\033[0;0m"<<std::endl; }
#endif
# include <chrono> // Adapted from rosconsole. Copyright (c) 2008, Willow Garage, Inc.
# define SVO_WARN_STREAM_THROTTLE(rate, x) \
    do { \
      static double __log_stream_throttle__last_hit__ = 0.0; \
      std::chrono::time_point<std::chrono::system_clock> __log_stream_throttle__now__ = \
      std::chrono::system_clock::now(); \
      if (__log_stream_throttle__last_hit__ + rate <= \
          std::chrono::duration_cast<std::chrono::seconds>( \
          __log_stream_throttle__now__.time_since_epoch()).count()) { \
        __log_stream_throttle__last_hit__ = \
        std::chrono::duration_cast<std::chrono::seconds>( \
        __log_stream_throttle__now__.time_since_epoch()).count(); \
        SVO_WARN_STREAM(x); \
      } \
    } while(0)
#endif


#define RED "\033[0;1;31m"
#define GREEN "\033[0;1;32m"
#define YELLOW "\033[0;1;33m"
#define BLUE "\033[0;1;34m"
#define PURPLE "\033[0;1;35m"
#define DEEPGREEN "\033[0;1;36m"
#define WHITE "\033[0;1;37m"
#define RED_IN_WHITE "\033[0;47;31m"
#define GREEN_IN_WHITE "\033[0;47;32m"
#define YELLOW_IN_WHITE "\033[0;47;33m"

#define TAIL "\033[0m"

inline void printFZ() {
  std::cout << GREEN
      "                       _oo0oo_                      \n"
      "                      o8888888o                     \n"
      "                      88\" . \"88                     \n"
      "                      (| -_- |)                     \n"
      "                      0\\  =  /0                     \n"
      "                   ___/‘---’\\___                   \n"
      "                  .' \\|       |/ '.                 \n"
      "                 / \\\\|||  :  |||// \\                \n"
      "                / _||||| -卍-|||||_ \\               \n"
      "               |   | \\\\\\  -  /// |   |              \n"
      "               | \\_|  ''\\---/''  |_/ |              \n"
      "               \\  .-\\__  '-'  ___/-. /              \n"
      "             ___'. .'  /--.--\\  '. .'___            \n"
      "         .\"\" ‘<  ‘.___\\_<|>_/___.’>’ \"\".          \n"
      "       | | :  ‘- \\‘.;‘\\ _ /’;.’/ - ’ : | |        \n"
      "         \\  \\ ‘_.   \\_ __\\ /__ _/   .-’ /  /        \n"
      "    =====‘-.____‘.___ \\_____/___.-’___.-’=====     \n"
      "                       ‘=---=’                      \n"
      "                                                    \n"
      "...Best wishes for you, never fried chicken, never bugs...\n"
      "...........................Amitabha.......................\n"
            << TAIL << std::endl;
}

inline void printKeyboard() {
  std::cout << BLUE
            << "   ┌─────────────────────────────────────────────────────────────┐\n"
               "   │┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐│\n"
               "   ││Esc│!1 │@2 │#3 │$4 │%5 │^6 │&7 │*8 │(9 │)0 │_- │+= │|\\ │`~ ││\n"
               "   │├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴───┤│\n"
               "   ││ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{[ │}] │ BS  ││\n"
               "   │├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤│\n"
               "   ││ Ctrl │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│\" '│ Enter  ││\n"
               "   │├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────┬───┤│\n"
               "   ││ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│Shift │Fn ││\n"
               "   │└─────┬──┴┬──┴──┬┴───┴───┴───┴───┴───┴──┬┴───┴┬──┴┬─────┴───┘│\n"
               "   │      │Fn │ Alt │         Space         │ Alt │Win│   HHKB   │\n"
               "   │      └───┴─────┴───────────────────────┴─────┴───┘          │\n"
               "   └─────────────────────────────────────────────────────────────┘\n"
            << TAIL << std::endl;
}

inline void printCNM() {
  std::cout << PURPLE
            << "  ┏┓　　　┏┓\n"
               "  ┏┛┻━━━┛┻┓\n"
               "  ┃　　　　　　  ┃\n"
               "  ┃　　　━　　　 ┃\n"
               "  ┃　＞　　　＜　┃\n"
               "  ┃　　　　　　　┃\n"
               "  ┃...　⌒　...  ┃\n"
               "  ┃　　　　　　　┃\n"
               "  ┗━┓　　　┏━┛\n"
               "      ┃　　　┃　\n"
               "      ┃　　　┃\n"
               "      ┃　　　┃\n"
               "      ┃　　　┃  Best Wishes\n"
               "      ┃　　　┃  Never BUG!\n"
               "      ┃　　　┃\n"
               "      ┃　　　┗━━━┓\n"
               "      ┃　　　　　　　┣┓\n"
               "      ┃　　　　　　　┏┛\n"
               "      ┗┓┓┏━┳┓┏┛\n"
               "        ┃┫┫　┃┫┫\n"
               "        ┗┻┛　┗┻┛"
            << TAIL << std::endl;
}
