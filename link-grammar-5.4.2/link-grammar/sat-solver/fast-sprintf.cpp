#include "fast-sprintf.hpp"

char* fast_sprintf(char* buffer, int num)
{
  // num is 0 or 1 most of the times
  if (num < 16) {
    *buffer++ = '0' + num;
  } else {
    do {
      *buffer++ = '0' + num % 16;
      num /= 16;
    } while (num > 0);
  }

  *buffer = '\0';

  return buffer;
}

char* fast_sprintf(char* buffer, const char* str)
{
  while ((*buffer++ = *str++))
    ;
  return buffer - 1;
}
