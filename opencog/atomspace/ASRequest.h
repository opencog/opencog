#ifndef _OPENCOG_ATOMSPACE_REQUEST_H
#define _OPENCOG_ATOMSPACE_REQUEST_H

#include <iostream>
#include <condition_variable>

#include <opencog/util/foreach.h>

#include <opencog/atomspace/AtomSpaceImpl.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/types.h>

using namespace std;

namespace opencog {
/** \addtogroup grp_atomspace
 *  @{
 */

/** \defgroup grp_atomspace_requess Asynchronous requests
 *
 * The AtomSpace class is essentially just a wrapper of the asynchronous
 * AtomSpaceAsync which returns ASRequest "futures" as well as allowing
 * thread-local caching of some requests. Functions in this
 * class will block until notified that they've been fulfilled by the
 * AtomSpaceAsync event loop.
 *
 *  @{
 */

class ASRequest {

protected:
    //! Overkill, but doing atomic operations on bool is not technically threadsafe
    //! Whaaaat ?? In what way?? why not ?? Huh? 
    mutable std::mutex complete_mutex;
    bool completed;

    //! For signalling that the request has been completed
    std::condition_variable complete_cond;
    //! For blocking while fulfilling the request
    mutable std::mutex the_mutex;

    AtomSpaceImpl* atomspace;
public:
    ASRequest() : completed(false) {};

    void set_atomspace(AtomSpaceImpl* as) {
        std::lock_guard<std::mutex> lock(the_mutex);
        atomspace = as;
    }
    //! We wrap the actual do_work thread so that it doesn't have
    //! to worry about obtaining the lock or notifying of completion
    void run() {
        std::lock_guard<std::mutex> lock(the_mutex);
        do_work();
        std::lock_guard<std::mutex> lock2(complete_mutex);
        completed = true;
        complete_cond.notify_all();
    }
    virtual void do_work() = 0;

    bool is_complete() {
        // Rely on separate mutex for complete, since we don't want to stall
        // if the do_work method takes a while.
        std::lock_guard<std::mutex> lock(complete_mutex);
        return completed;
    }
};


/** @}*/
/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_REQUEST_H
