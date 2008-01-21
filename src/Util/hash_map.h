#ifndef _UTIL_HASH_MAP_H
#define _UTIL_HASH_MAP_H

#include <string>
#include <ext/hash_map>

namespace Util {
    using __gnu_cxx::hash_map;
    using __gnu_cxx::hash;

}

namespace __gnu_cxx{
    template<> struct hash< std::string > {
        size_t operator()( const std::string& x ) const {
            return hash< const char* >()( x.c_str() );
        }
    };
}


#endif
