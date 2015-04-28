//
#ifndef _OPENCOG_PYTHON_SCHEME_H
#define _OPENCOG_PYTHON_SCHEME_H

#include <string>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{

/** For easier wrapping by Cython */
std::string eval_scheme(AtomSpace& as, const std::string &s);
Handle eval_scheme_h(AtomSpace& as, const std::string &s);

} // namespace opencog

#endif // _OPENCOG_PYTHON_SCHEME_H

