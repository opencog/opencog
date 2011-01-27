/** BaseInstantiationRule.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Authors: Nil Geisweiller
 *          Ari Heljakka
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


#ifndef _OPENCOG_BASEINSTANTIATIONRULE_H
#define _OPENCOG_BASEINSTANTIATIONRULE_H

#include <boost/lexical_cast.hpp>

#include "../RuleProvider.h"
#include "../Rule.h"


namespace haxx
{
    /// \todo This data must persist even if the BITNodeRoot is deleted.
    
    /// \todo move this info to the definition...
    /// Given an inference step, the key pHandle corresponds to the
    /// conclusion of the step, the value pHandleSeq corresponds to
    /// the premises.
    extern std::map<pHandle,pHandleSeq> inferred_from;
    /// the key pHandle is the conclusion (just as above) and Rule* is
    /// the rule that has been applied to create it
    extern std::map<pHandle,opencog::pln::RulePtr> inferred_with;
}

namespace opencog { namespace pln {

/**
 * Base class of instantiation rules. That is given instances A in
 * output, look for
 *
 * quantifierLink
 *     ListLink
 *     X1
 *     ...
 *     Xn
 *     Body(X1, Xn)
 *
 * so that there exist X1=A1, ..., Xn=An and Body(X1/A1, ..., Xn/An)=A
 *
 * A.TV is calculated with the functor FormulaType.
 */
template<typename FormulaType>
class BaseInstantiationRule : public Rule
{
protected:
    mutable FormulaType formula;

    /** The link that this rule has been instantiated for.
     */
    pHandle quantifierLink;
    Type quantifierType;

    inline void makeName(pHandle h) {
        //append the pHandle quantifierLink so that the rule name is unique
        name += boost::lexical_cast<std::string>(quantifierLink);
    }
public:

    //! Obsolete is set to true if the Handle this rule is based off of is
    //! removed from the AtomSpace.
    bool obsolete;

    //! This version is needed for adding via add/remove signals, because
    //! ASW access from add/remove handlers is not allowed.
    BaseInstantiationRule(Type _quantifierType,
                          pHandle _quantifierLink,
                          AtomSpaceWrapper *_asw, const std::string& _name)
        : Rule(_asw, false, false, _name), quantifierLink(_quantifierLink)
    {
        quantifierType = _quantifierType;
        makeName(quantifierLink);
        inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ATOM))));
        obsolete = false;
    }

    BaseInstantiationRule(pHandle _quantifierLink,
                          AtomSpaceWrapper *_asw, const std::string& _name)
        : Rule(_asw, false, false, _name), quantifierLink(_quantifierLink)
    {
        quantifierType = asw->getType(quantifierLink);
        //append the pHandle quantifierLink so that the rule name is unique
        makeName(quantifierLink);
        inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ATOM))));
        obsolete = false;
    }

    BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const
    {
        // Only for use as a Generator, so doesn't need a compute method.
        assert(0);
        return Vertex(PHANDLE_UNDEFINED);
    }

    setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const {
        return Rule::setOfMPs();
    }

    bool validate2(MPs& args) const { return true; }

    Btr<std::set<BoundVertex > > attemptDirectProduction(meta outh,
                                                         bool fresh = true) const {
        if (asw->isSubType(_v2h(*outh->begin()), quantifierType) ||
            asw->isSubType(_v2h(*outh->begin()), FW_VARIABLE_NODE))
            return Btr<std::set<BoundVertex > >();

        NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME|NMP_NODE_TYPE_NAME);
        printer.print(vtree(outh->begin()));

        cprintf(3,"FindMatchingUniversal...\n");
        Btr<ModifiedBoundVTree> i = FindMatchingUniversal(outh, quantifierLink, asw);
        cprintf(3,"FindMatchingUniversal OK!\n");
        
        if (!i) return Btr<std::set<BoundVertex > >();

        Btr<std::set<BoundVertex > > ret(new std::set<BoundVertex >);
    
        MPs ret1;
        typedef std::pair<pHandle,vtree> phvt;
        DeclareBtr(bindingsT, pre_binds);

        foreach(phvt vp, *i->bindings) {
            // Never bind a VariableNode to ATOM. That cannot be used
            // in a real Atom. This check can't just be done on outh
            // (the target), because that would stop ATOM in the
            // target from matching to something _valid_ in the
            // ForAllLink. Though it doesn't actually support that
            // anyway, see the following.
            //
            //! @todo Note that if target contains ATOM, the algo
            //! won't fill in something from the ForAllLink that was
            //! more specific. This would be useful for FC.
            if (std::find(vp.second.begin(), vp.second.end(),
                          Vertex((pHandle)ATOM)) != vp.second.end()) {
                return Btr<std::set<BoundVertex > >();
            }

            (*pre_binds)[vp.first] = make_real(vp.second);
            printer.print(vp.first);
            cprintf(0,"=");
            printer.print((*pre_binds)[vp.first]);
        }
        
        // root atom of the body
        BBvtree rootAtom(new BoundVTree(*i, pre_binds)); 

        //    bind_Bvtree(rootAtom, *i->bindings);
        //    pHandle topologicalStub = asw->addAtom(*rootAtom, TruthValue::TRIVIAL_TV(), false);
        //
        //    pHandle ret_h = asw->addLink(asw->getType(topologicalStub),
        //                                 asw->getOutgoing(topologicalStub),
        //                                 asw->getTV(i->original_handle), fresh);
        
        bind_Bvtree(rootAtom, *i->bindings);

        TruthValue* retTV = formula.compute(TVSeq(1, asw->getTV(i->original_handle)));

        pHandle ret_h = asw->addAtom(*rootAtom, *retTV, false);

        delete(retTV);
    
        ret->insert(BoundVertex(ret_h, pre_binds));
        //    ret->insert(BoundVertex(ret_h, NULL));

        /*  haxx::bitnoderoot->inferred_with[ret_h] = (Rule*)(int)this;
            if (haxx::bitnoderoot->inferred_from[ret_h].empty()) //comes here often, we want the link only once
            haxx::bitnoderoot->inferred_from[ret_h].push_back(hForAllLink);
        */
        haxx::inferred_with[ret_h] = referenceRuleProvider().findRule(name);
        if (haxx::inferred_from[ret_h].empty()) //comes here often, we want the link only once
            haxx::inferred_from[ret_h].push_back(quantifierLink);
        
        // Necessary since this is not the only way of producing the Atom.
        //reviseVersionedTVs(asw->fakeToRealHandle(_v2h(ret_h)).first);

        return ret;
    }
};

}} // namespace opencog { namespace pln {

#endif // _OPENCOG_BASEINSTANTIATIONRULE_H
