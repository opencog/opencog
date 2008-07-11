#ifdef _PLN_H
#define _PLN_H

// What does this do because I don't know... GCC ignores it
#ifdef WIN32
#pragma warning( disable : 4786)
#pragma warning( disable : 4503)
#endif

// C includes
#include <types.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// C++ includes
#include <set>
#include <algorithm>
#include <stack>
#include <string>
#include <map>
#include <stack>

// Boost includes
#include <boost/smart_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>

// OpenCog includes
#include <AtomSpace.h>
//#ifndef USE_PSEUDOCORE
	#include <Link.h>
	#include <Atom.h>
	#include <Node.h>
	#include <TLB.h>
//#endif
#include <classes.h>
#include <SimpleTruthValue.h>

// PLN includes
//#include "CoreWrapper.h"
//#include "Rule.h"
//#include "iAtomTableWrapper.h"

// PLN utility includes
//#include "PLNUtils.h"
//#include "util/log.h"
//#include <utils2.h>

// The fixed length of a pattern
#define PLN_CONFIG_PATTERN_LENGTH 13
#define PLN_CONFIG_FIM 1
#define PLN_CONFIG_COLLAPSE_LIST_LINKS 0

#define foreach BOOST_FOREACH
#define Btr boost::shared_ptr

typedef unsigned long ulong;
enum FitnessEvalutorT { DETERMINISTIC, RANDOM, SOFTMAX };
enum MetaProperty { NONE, STRENGTH, CONFIDENCE, STRENGTH_CONFIDENCE, LTI, STI };
#endif // _PLN_H
