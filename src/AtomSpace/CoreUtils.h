/**
 * CoreUtils.h
 * 
 * Module for including any core-specific common utilities
 *
 * Copyright(c) 2007 Vettalabs
 * All rights reserved.
 */

#ifndef _CORE_UTILS_H_
#define _CORE_UTILS_H_

#include "types.h"
#include "HandleMap.h"
#include "exceptions.h"

class CoreUtils
{
public:    

    /**
     * This method is used to translate an old handle to a new one mapped
     * in a hash table.
     *
     * @param Handle which will be translated.
     * @param Table that maps from old to new handles.
     */
    static void updateHandle(Handle *, HandleMap<Atom *> *) throw (RuntimeException);
    
    /**
     * Handle sort criterion used by qsort. It returns a negative value,
     * zero or a positive value if the first argument is respectively
     * smaller than, equal to, or larger then the second argument.
     *
     * @param The pointer to the first handle element.
     * @param The pointer to the second handle element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger than the
     * second argument.
     */
    static int handleCompare(const void*, const void*);

    /**
     * Returns a negative value, zero or a positive value if the first 
     * argument is respectively smaller than, equal to, or larger than 
     * the second argument.
     *
     * @param The first handle element.
     * @param The second handle element.
     * @return A negative value, zero or a positive value if the first
     * argument is respectively smaller than, equal to, or larger then the
     * second argument.
     */
    static int compare(Handle, Handle);
    
    class HandleComparison {
        public:
        bool operator()(const Handle& h1, const Handle& h2) const;
    };
};    


#endif
