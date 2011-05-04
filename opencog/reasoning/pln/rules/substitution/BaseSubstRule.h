/** BaseSubstRule.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_BASESUBSTRULE_H
#define _OPENCOG_BASESUBSTRULE_H

namespace opencog { namespace pln {

static const std::string SubstStr = "Subst";

/**
 * Rule for substituting an atom A by an atom B inside an
 * expression given the relation between A and B
 *
 * R <TV1>
 *     A
 *     B
 * C[A] <TV2>
 * |-
 * C[A/B] <TV3>
 *
 * where TV3 is determined by the formula template argument
 */
template<typename SubstFormula>
class BaseSubstRule : public GenericRule<SubstFormula> {
    typedef GenericRule<SubstFormula> super;
protected:
    Type relationType; // relation between A and B

    bool LByR; // if true (the default) then rule substitute the left
               // term (A) by the right term (B) in the C[A]. Other it
               // sustitute the right term (B) by the left term (A) in
               // the expression C[B].

    meta i2oType(const VertexSeq& h) const
    {
        OC_ASSERT(h.size() == 2);
        pHandle R = _v2h(h[0]);
        pHandle C = _v2h(h[1]);
        OC_ASSERT(super::asw->getArity(R) == 2);
        OC_ASSERT(super::asw->getType(R) == relationType);
        pHandle A = super::asw->getOutgoing(R,0);
        pHandle B = super::asw->getOutgoing(R,1);
        OC_ASSERT(A != PHANDLE_UNDEFINED);
        OC_ASSERT(C != PHANDLE_UNDEFINED);
        
        // Make a vtree out of C
        meta C_mp = meta(new vtree(h[1]));
        // Make it virtual (to get the structure of the link, not just
        // the handle)
        meta ret = ForceAllLinksVirtual(C_mp);

        /// @todo What if child has a different structure to parent?
        if(LByR)
            std::replace(ret->begin(), ret->end(), Vertex(A), Vertex(B));
        else
            std::replace(ret->begin(), ret->end(), Vertex(B), Vertex(A));

        return ret;
    }

    bool validate2(Rule::MPs& args) const { return true; }

    TVSeq formatTVarray(const VertexSeq& premiseArray) const
    {
        OC_ASSERT(premiseArray.size() == 2);
        TVSeq tvs;
        tvs.push_back(super::asw->getTV(_v2h(premiseArray[0]))); // R <TV1>
		tvs.push_back(super::asw->getTV(_v2h(premiseArray[1]))); // C <TV2>
        return tvs;
    }

public:
    BaseSubstRule(AtomSpaceWrapper* _asw, Type _relationType, bool _LByR = true) 
        : super(_asw, false, SubstStr), relationType(_relationType), LByR(_LByR)
    {
        OC_ASSERT(classserver().isA(relationType, LINK));
        std::string relationName = classserver().getTypeName(relationType);
        super::name = relationName.substr(0, relationName.find("Link")) 
            + super::name + (LByR? "LByR" : "RByL") + "Rule";

        super::inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)relationType,
                                                               mva((pHandle)ATOM),
                                                               mva((pHandle)ATOM)))));
        super::inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ATOM))));
    }
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        /** For simplicity (but sacrificing applicability),
            FW_VARs cannot be replaced with children structs.
            Links are assumed not inheritable either.
        */    
        // Skipping links prevents an assert error in
        // BITNode::tryClone(); skipping FW_VARS is required to avoid
        // trying to replace FW_VARS already created by this Rule,
        // and/or a crash. -- JaredW

        // There are two extra restrictions I added to make the BIT
        // search efficient enough: Also skip targets that are
        // InheritanceLinks or similar; whether they are an actual
        // target or the child of another SSR1 BITNode, they can be
        // handled better via Deduction. This is also for
        // inference-control efficiency.

        // Also, only substitute ConceptNodes, not PredicateNodes (a
        // heuristic; Inheritance between other types of Nodes isn't
        // necessarily used)

        pHandle top = _v2h(*outh->begin()); // Top of the target vtree

        if (outh->begin().number_of_children() != 2
            ||  super::asw->isSubType(top, FW_VARIABLE_NODE)
            ||  super::asw->isSubType(top, INHERITANCE_LINK)
            ||  super::asw->isSubType(top, SUBSET_LINK)
            ||  super::asw->isSubType(top, INTENSIONAL_INHERITANCE_LINK)
            ||  super::asw->isSubType(top, IMPLICATION_LINK)
            ||  super::asw->isSubType(top, EXTENSIONAL_IMPLICATION_LINK))
            return Rule::setOfMPs();

        /*  puts("X1");
            rawPrint(*outh,0,0);
            puts("X2");*/
    
        Rule::setOfMPs ret;


        //  set<atom> child_nodes;
        //  find_child_nodes(outh, child_nodes);
    
        Vertex child = CreateVar(super::asw);
    
        // The input is: an inheritance from any part of the output atom,
        // and a version of the output atom that has that atom in it.
        for(tree<Vertex>::pre_order_iterator i = outh->begin(); i != outh->end(); i++) {
            //  for (set<atom>::iterator i = child_nodes.begin(); i != child_nodes.end(); i++)

            /*      puts("-");
                    printAtomTree(outh,0,0);
            */
            // if this vertex is a link type or FW_VAR, skip it
            /*if ( asw->isSubType(_v2h(*i), LINK) || asw->isSubType(_v2h(*i),
              FW_VARIABLE_NODE)*/
            // Only substitute ConceptNodes
            if (!super::asw->isSubType(_v2h(*i), CONCEPT_NODE))
                continue;

            // Put the FW_VAR (child) into the templated atom
            Vertex old_i = *i;
            *i = child;
            BBvtree templated_atom1(new BoundVTree(*outh));
            *i = old_i;     
        
            // Put the FW_VAR into an InheritanceLink
            BBvtree inhPattern1;
            if (LByR)
                inhPattern1 = BBvtree(new BoundVTree(mva((pHandle)relationType,
                                                         mva(child), mva(*i))));
            else
                inhPattern1 = BBvtree(new BoundVTree(mva((pHandle)relationType,
                                                         mva(*i), mva(child))));
            
            Rule::MPs ret1;
            ret1.push_back(inhPattern1);
            ret1.push_back(templated_atom1);
        
            /*      puts("X");
                    rawPrint(*outh,0,0);
                    puts("-");
                    rawPrint(*templated_atom1,0,0);
                    puts("-");
                    rawPrint(*inhPattern1,0,0);
                    puts("-");*/
        
            ForceAllLinksVirtual(inhPattern1);
            ForceAllLinksVirtual(templated_atom1);
        
            overrideInputFilter = true;
            
            ret.insert(ret1);
        }
        return ret;
    }

    /// @todo Nil: SimSubstRule1 is missing targetTemplate so I'm not
    /// including it either assuming it's OK for now

};

}} // namespace opencog { namespace pln {


#endif // _OPENCOG_BASESUBSTRULE_H
