/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef PLNATOM_H
#define PLNATOM_H

#define MetaPredicate atom

//! @todo use class server to find out what this values should actually be
#define RESTRICTOR   158
#define INSTANCEOF_R 159
#define HAS_R        160
#define IN_R         161

#define TYPED        162

#define __EQUALS     163
#define __EQUALS_N   164
#define __INSTANCEOF_N 165
#define __INDEXER    166
#define __INDEX1     167
#define __INDEX2     168
#define __INDEX3     169
#define __INDEX4     170

#define __OR         171
#define __AND        172
#define __NOT        173

#include "iAtomSpaceWrapper.h"
#include <boost/smart_ptr.hpp>

namespace opencog { namespace pln {

struct atom;
typedef std::pair<std::string, atom> subst;
struct lessatom;
struct less_subst;

/** atom structure. An obsolete mess from before vtree.
 * Also called a MetaPredicate or MP in the rules. But not anymore, because
 * MP is now based on vtree.
 *
 * @todo remove need for the atom class in the unification methods.
 */
struct atom
{
public:
    mutable pHandle handle;
    mutable std::map<std::string,atom>* bindings;

    Type T;
    int arity;
    std::string name;

    std::vector<boost::shared_ptr<atom> > hs;
    
    mutable std::set<subst>* forbiddenBindings;

    bool operator<(const atom& rhs) const;
    bool operator!=(const atom& rhs) const { return (*this)<rhs || rhs<(*this); }

    void setHandle(pHandle h);   

    explicit atom(pHandle h);
    atom();
    atom(const atom& rhs);
    atom(Type _T, std::string _name);
    atom(Type _T, int _arity, ...);
    atom(Type _T, std::vector<boost::shared_ptr<atom> > hs);
    atom(const tree<boost::shared_ptr<atom> >& a,
        tree<boost::shared_ptr<atom> >::iterator parent_node,
        bool root = true);
    atom(const vtree& a, vtree::iterator parent_node, bool root = true);

    /** Create from an integer pattern of a pre-defined length. Doesn't change the ownership
        of the array.
    */

    //atom(unsigned int*);
    ~atom();

    bool well_formed() const;

    /// Probably expensive operation.
    void SetOutgoing(pHandleSeq _hs);

    /// Create a copy of this wrapper into the core, regardless whether this
    /// was created from a core handle.
    pHandle attach(iAtomSpaceWrapper* core) const;
    void detach() const;

    /// Copy this as an integer array into the dest array
    int asIntegerArray(unsigned int* dest, unsigned int patlen, std::map<atom,int,lessatom>& node2pat_id,
                        unsigned int& next_free_pat_id, int index = 0) const;

    MetaPredicate* getMeta() const;
    Handle getHandle() const;

    bool operator==(const atom& rhs) const;

    Type execType() const;
    std::vector<boost::shared_ptr<atom> > execOutTree() const;
    bool matchType(const atom& rhs) const;
    bool matchType(Type rhsT) const;
    std::string execName() const { return name; }

    bool containsVar() const;
    bool containsFWVar() const;

    bool forbidLastSubstitution() const;

    /** Metapredicate operation */
    bool operator()(pHandle h) const;

    static bool ValidMetaPredicate(Type T);

    static pHandleSeq convertVector(const std::vector<boost::shared_ptr<atom> >& hs,
                                        iAtomSpaceWrapper* table);

    void substitute(pHandle rhs, std::string varname);
    void substitute(pHandle dest, atom src);
    void substitute(const atom& dest, const atom& src);
    void substitute(const atom& rhs, std::string varname);
    
    //! @todo Write proper visitor object!
    tree< boost::shared_ptr<atom> > maketree() const; //tree<atom>& dest);
    
    vtree makeHandletree(iAtomSpaceWrapper* table, bool fullVirtual=false) const; //tree<atom>& dest);

    /// Actualize the substitution from bindings, delete forbiddenBindings, on new instance..
    void getWithActualizedSubstitutions(atom& target) const;
    pHandle bindHandle(iAtomSpaceWrapper* table) const;

    void extractVars(std::set<std::string>& vars) const;
    void extractFWVars(std::set<std::string>& vars) const;

};

struct lessatom : public std::binary_function<atom, atom, bool>
{
    bool operator()(const atom& lhs, const atom& rhs) const;
};

struct lessatom_ignoreVarNameDifferences : public std::binary_function<atom, atom, bool>
{
    bool operator()(const atom& lhs, const atom& rhs) const
    {
        int diff = nodedifference(lhs, rhs);

        if (diff < 0)
            return true;
        else
            return false;
    }

    int nodedifference(const atom& lhs, const atom& rhs) const
    {
        //hack: ATOM type equivals "ANY"
        if (  (lhs.T == ATOM && rhs.T != FW_VARIABLE_NODE)
            ||(rhs.T == ATOM && lhs.T != FW_VARIABLE_NODE))
            return 0;

        int lhsize = (int)lhs.hs.size();
        int rhsize = (int)rhs.hs.size();

        if (lhs.T < rhs.T)
            return -1;
        if (lhs.T > rhs.T)
            return 1;


        if (lhsize < rhsize)
            return -1;
        if (lhsize > rhsize)
            return 1;
            
        if ((lhs.T != VARIABLE_NODE) && rhs.T != FW_VARIABLE_NODE)
        {
            if (lhs.name < rhs.name)
                return -1;
            if (lhs.name > rhs.name)
                return 1;
        }

        if (lhsize == 0)
            return 0;

        /// Otherwise equal as nodes, with same arity. Then, check outgoing set:

        for (int i = 0; i < lhsize; i++)
        {
            int diff = nodedifference(*lhs.hs[i], *rhs.hs[i]);

            if (diff < 0)
                return -1;
            if (diff > 0)
                return 1;
        }

        return 0;
    }
};

void printAtomTree(const atom& a, int level = 0, int LogLevel = 5);

#define equal_atom_ignoreVarNameDifferences(a,b) \
    (!lessatom_ignoreVarNameDifferences()(a,b) && \
    !lessatom_ignoreVarNameDifferences()(b,a))

template<int L>
class atom_print
{
public:
    void operator()(const atom& rhs) { printAtomTree(rhs, 0,L); }
};


template<int L>
class handle_print
{
public:
    void operator()(pHandle rhs) { 
    //  printTree(rhs, 0,L); 
    }
};

typedef std::set<atom, lessatom> atomset;

void getAtomTreeString(const atom& a, std::string& outbuf);

void VariableMPforms(const atom& src, std::set<atom, lessatom_ignoreVarNameDifferences>& res,
                     std::set<subst>* forbiddenBindings);
bool getLargestIntersection2(const std::set<atom,lessatom>& keyelem_set,
                            const std::vector<pHandle>& link_set, std::vector<boost::shared_ptr<atom> >& result);

atom* neBoundVertexWithNewType(Handle h, Type T);

}} // ~namespace opencog::pln

#endif
