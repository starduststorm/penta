#pragma once
#include <stdint.h>

// A monotonically increasing build number derived from the compile time:
// seconds since 2000-01-01 UTC, computed from __DATE__/__TIME__ at compile
// time. Boards compare these to decide who has newer firmware.
namespace buildid {
constexpr int monthFrom(const char *d) {
  return (d[0] == 'J' && d[1] == 'a') ? 1 : (d[0] == 'F') ? 2 : (d[0] == 'M' && d[2] == 'r') ? 3
       : (d[0] == 'A' && d[1] == 'p') ? 4 : (d[0] == 'M') ? 5 : (d[0] == 'J' && d[2] == 'n') ? 6
       : (d[0] == 'J') ? 7 : (d[0] == 'A') ? 8 : (d[0] == 'S') ? 9 : (d[0] == 'O') ? 10
       : (d[0] == 'N') ? 11 : 12;
}
constexpr int num(char c) { return c == ' ' ? 0 : c - '0'; }
constexpr uint32_t daysBefore(int y, int m) { // days from 2000-01-01 to y-m-01
  uint32_t d = 0;
  for (int yy = 2000; yy < y; yy++) d += (yy % 4 == 0 && (yy % 100 != 0 || yy % 400 == 0)) ? 366 : 365;
  constexpr int ml[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  for (int mm = 1; mm < m; mm++) d += ml[mm - 1] + ((mm == 2 && y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 1 : 0);
  return d;
}
constexpr uint32_t epoch(const char *date, const char *time) {
  // date "Mmm dd yyyy", time "hh:mm:ss"
  int y = (date[7] - '0') * 1000 + (date[8] - '0') * 100 + (date[9] - '0') * 10 + (date[10] - '0');
  int m = monthFrom(date);
  int d = num(date[4]) * 10 + num(date[5]);
  int hh = num(time[0]) * 10 + num(time[1]), mi = num(time[3]) * 10 + num(time[4]), ss = num(time[6]) * 10 + num(time[7]);
  return ((daysBefore(y, m) + d - 1) * 86400u) + hh * 3600u + mi * 60u + ss;
}
} // namespace buildid

// scripts/build_epoch.py stamps each build with the unix time; the __DATE__
// fallback only kicks in if the script didn't run (it would then only change
// when this translation unit is recompiled).
#if __has_include("build_epoch.h")
#include "build_epoch.h"
#define BUILD_EPOCH ((uint32_t)BUILD_EPOCH_UNIX)
#else
#define BUILD_EPOCH (buildid::epoch(__DATE__, __TIME__))
#endif
