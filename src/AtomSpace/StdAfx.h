// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//

#if !defined(AFX_STDAFX_H__7AAB3E55_1D47_4992_8887_3AA8E66EA1D6__INCLUDED_)
#define AFX_STDAFX_H__7AAB3E55_1D47_4992_8887_3AA8E66EA1D6__INCLUDED_

#if 0

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#pragma warning( disable : 4786)
#pragma warning( disable : 4503)


#include "../common/core/types.h"

//New

float max(float a, float b);
float min(float a, float b);

int round(float x);

typedef unsigned int nmtype;

#include "boost/smart_ptr.hpp"

#include "Import.h"
#include "Interpreter.h"

//MEMORY MANAGE LISTS
#include "Internal.h"

#include "Numerics.h"
#include "../common/core/types.h"
#include "NMCore.h"
#include <stdio.h>
#include <set>
#include <algorithm>
#include <stack>

/* End of PseudoCore */

#include <stdarg.h>
#include <string>
#include <map>
#include <stack>

#include "tree.h"

using Util::tree;
using namespace std;

#include "utils.h"
//#include "simconfig.h"

#include "../reasoning/util/XMLNode.h"

//#include "PTLFormulas.h"

#include "utils/utils2.h"
#include "../reasoning/util/PTLutils.h"
//#include "TreeUtils.h"

using namespace pseudocore;

/// Parameter: The fixed length of a pattern


//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif

#endif // !defined(AFX_STDAFX_H__7AAB3E55_1D47_4992_8887_3AA8E66EA1D6__INCLUDED_)
