/**
 * PredicateHandleSet.h
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Sun Aug 26 12:51:18 BRT 2007
 */

#ifndef PREDICATEHANDLESET_H
#define PREDICATEHANDLESET_H

#include <set>
#include <string>
#include <opencog/atomspace/types.h>

using namespace opencog;

namespace behavior {

// TODO: this class should extend std::set
class PredicateHandleSet {

    private:

        std::set<Handle> handles;

    public:

        // ***********************************************/
        // Constructors/destructors

        virtual ~PredicateHandleSet();
        PredicateHandleSet();

        // ***********************************************/
        // API

        void insert(const Handle &handle);
        void clear();
        const std::set<Handle> &getSet() const;
        int hashCode();
        int getSize() const;
	bool empty() const;
        bool equals(const PredicateHandleSet &other) const;
        std::string toString() const;
        virtual PredicateHandleSet &operator=(const PredicateHandleSet& other);
        virtual bool operator==(const PredicateHandleSet& other) const;
        virtual bool operator<(const PredicateHandleSet& other) const;
}; // class
}  // namespace

#endif
