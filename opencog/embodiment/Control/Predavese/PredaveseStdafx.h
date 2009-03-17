#ifndef PREDAVESE_STDAFX_H
#define PREDAVESE_STDAFX_H

// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN        // Exclude rarely-used stuff from Windows headers
#endif

#include <stdio.h>
#ifdef WIN32
#include <tchar.h>
#endif

#include <string>
#include <map>
#include <vector>
#include <set>
#include <boost/variant.hpp>

#ifdef WIN32
#include <boost/foreach.hpp>
#else 
//#include "foreach.hpp"
#include <LADSUtil/foreach.h>
#endif

#ifdef WIN32
#define foreach BOOST_FOREACH
#endif

#include <algorithm>

#endif
