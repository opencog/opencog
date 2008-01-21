#ifndef _UTIL_PLATFORM_H_
#define _UTIL_PLATFORM_H_

#include <stdio.h>
#include <string.h>
#include <assert.h>

namespace Util {
  struct eqstr {
    bool operator()(char *s1, char *s2) const {
      return strcmp(s1, s2) == 0;
    } 
  };

  struct eqconststr {
    bool operator()(const char *s1, const char *s2) const {
      return strcmp(s1, s2) == 0;
    } 
  };

  struct eqint {
    bool operator()(int s1, int s2) const {
      return s1 == s2;
    } 
  };
} //~namespace Util

#ifdef WIN32

#include <hash_set>
#include <hash_map>
#include <functional>

/// Windows headers define ATOM for other purposes, so we temporarily undef it.
#undef ATOM
#include <windows.h>
#include <Mmsystem.h>
#define ATOM 0

#include <time.h>

namespace Util {
  using namespace std;

  typedef unsigned long ulong;
#define M_PI 3.14159265358979323846 
#define DATA_DIR "c:\\NMDATA"

  int round(float x);
  char * __strtok_r(char *s1, const char *s2, char **lasts);
  int gettimeofday(struct timeval* tp, void* tzp);
  void usleep(unsigned int useconds);
  int __getpid(void);
  double rint(double nr);
  int __dup2(int, int);

  unsigned sleep(unsigned seconds);

#define stl_hash_ul(x) hash<unsigned long>()(x)
  typedef hash<int> int_hash;
  typedef hash<const char*> const_char_hash;
  typedef hash_map<const char*, int, const_char_hash, eqconststr> ConstCharPointerIntHashMap;
  typedef hash_map<int, const char*, int_hash, eqint> IntConstCharPointerHashMap;
  typedef hash_map<const char*, void*, const_char_hash, eqconststr> ConstCharPointerVoidHashMap;    
  typedef hash_map<int,int> Int2IntHashMap;
  typedef hash_set<int> Int2IntHashSet;
} //~namespace Util
#else // !WIN32

#include <ext/hash_set>
#include <ext/hash_map>
#include <ext/functional>
#define stl_hash_ul(x) __gnu_cxx::hash<unsigned long>()(x)
#define LPWSTR char*

namespace Util {
  typedef __gnu_cxx::hash<int> int_hash;
  typedef __gnu_cxx::hash<const char*> const_char_hash;
  typedef __gnu_cxx::hash_map<const char*, int, const_char_hash, eqconststr> ConstCharPointerIntHashMap;
  typedef __gnu_cxx::hash_map<int, const char*, int_hash, eqint> IntConstCharPointerHashMap;
  typedef __gnu_cxx::hash_map<const char*, void*, const_char_hash, eqconststr> ConstCharPointerVoidHashMap;    
  typedef __gnu_cxx::hash_map<int,int> Int2IntHashMap;
  typedef __gnu_cxx::hash_set<int> Int2IntHashSet;

  //using namespace __gnu_cxx;
  using __gnu_cxx::hash_map;
  using __gnu_cxx::hash;
  //using __gnu_cxx::select1st;
  //using __gnu_cxx::select2nd;
  using std::make_pair;
  using namespace std;
} //~namespace Util

#endif // WIN32!


#endif //_PLATFORM_H_
