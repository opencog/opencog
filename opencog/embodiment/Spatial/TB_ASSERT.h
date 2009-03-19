#ifndef TBASSERT_H
#define TBASSERT_H

#include "util/exceptions.h"

// DEPRECATED
// TB_ASSERT - do not throw exceptions and have trace info
//#define TB_ASSERT(x) if (!(x)) MAIN_LOGGER.log(Util::Logger::ERROR, "TB_ASSERT - %s",  #x)

#define TB_ASSERT(x) if (!(x)) opencog::cassert(TRACE_INFO, "TB_ASSERT - %s",  #x)

#endif
