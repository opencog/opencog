#ifndef _OPENCOG_ATOMSPACE_ASYNC_H
#define _OPENCOG_ATOMSPACE_ASYNC_H

#include <opencog/atomspace/AtomSpaceImpl.h>

class AtomSpaceAsyncUTest;

namespace opencog {
/** \addtogroup grp_atomspace
 *  @{
 */

class AtomSpace;
class SavingLoading;

class AtomSpaceAsync
{
    friend class ::AtomTableUTest;
    friend class ::AtomSpaceAsyncUTest;
    friend class AtomSpaceBenchmark;
    friend class AtomSpace;
    friend class SavingLoading;
    friend class PersistModule;

    AtomSpaceImpl atomspace;
public: 
    AtomSpaceImpl& getImpl() { return atomspace; }
    const AtomSpaceImpl& getImplconst() const { return atomspace; }

    AtomSpaceAsync() {}
    ~AtomSpaceAsync() {}

    int get_counter() { return 0; }
    bool isQueueEmpty() { return true; }

    //--------------
    // These functions are not run in the event loop, but may need guards as
    // a result. They are also too low level for a network API and should only
    // be used by modules that know what they are doing.
    inline void registerBackingStore(BackingStore *bs) { atomspace.registerBackingStore(bs); }
    inline void unregisterBackingStore(BackingStore *bs) { atomspace.unregisterBackingStore(bs); }

    // TODO XXX FIXME convert to boost::signals2 ASAP for thread safety.
    boost::signals::connection addAtomSignal(const AtomSignal::slot_type& function) {
        return atomspace.addAtomSignal().connect(function);
    }
    boost::signals::connection removeAtomSignal(const AtomPtrSignal::slot_type& function) {
        return atomspace.removeAtomSignal().connect(function);
    }
    boost::signals::connection AVChangedSignal(const AVCHSigl::slot_type& function) {
        return atomspace.AVChangedSignal().connect(function);
    }
    boost::signals::connection TVChangedSignal(const TVCHSigl::slot_type& function) {
        return atomspace.TVChangedSignal().connect(function);
    }

    //--------------
    inline AttentionBank& getAttentionBank()
    { return atomspace.getAttentionBank(); }

    inline const AttentionBank& getAttentionBankconst() const
    { return atomspace.getAttentionBankconst(); }

    const AtomTable& getAtomTable() { return atomspace.getAtomTable(); };
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_ATOMSPACE_ASYNC_H
