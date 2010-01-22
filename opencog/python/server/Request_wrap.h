#ifndef _OPENCOG_REQUEST_WRAP_H
#define _OPENCOG_REQUEST_WRAP_H

#include <boost/python/wrapper.hpp>

#include <opencog/server/Request.h>

using namespace boost::python;
using namespace opencog;

/** Exposes the Request class. */
void init_Request_py();

/** A class wrapper of the Request class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct RequestWrap : Request, wrapper<Request>
{
    // Pure virtual functions.

    bool execute(void);

    // Non-pure virtual functions.

    void send(const std::string& msg) const;
    void default_send(const std::string& msg) const;
    void setRequestResult(RequestResult*);
    void default_setRequestResult(RequestResult*);
    RequestResult *getRequestResult(void);
    RequestResult *default_getRequestResult(void);
    void setParameters(const std::list<std::string>& params);
    void default_setParameters(const std::list<std::string>& params);
    void addParameter(const std::string& param);
    void default_addParameter(const std::string& param);
};

#endif // _OPENCOG_REQUEST_WRAP_H
