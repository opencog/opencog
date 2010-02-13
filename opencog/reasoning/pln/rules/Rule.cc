/*
 *  Rule.cc
 *
 *  Created by Cesar Marcondes.
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

#include <opencog/util/platform.h>
#include "../PLN.h"
#include "Rule.h"
#include "Rules.h"
#include "../AtomSpaceWrapper.h"
#include "../PLNatom.h"
#include "../BackInferenceTreeNode.h"

using namespace std;

namespace opencog {
namespace pln {

Rule::Rule(opencog::pln::AtomSpaceWrapper *_asw,
           bool _freeInputArity,
           bool _composer,
           std::string _name)
  : RULE_INPUT_ARITY_MAX(15), freeInputArity(_freeInputArity),
    asw(_asw), composer(_composer), priority(10.0f), name(_name) 
{ }

Rule::Rule() : RULE_INPUT_ARITY_MAX(15) { }

Rule::~Rule()
{
/*  for (map<atom, setOfMPs, lessatom_ignoreVarNameDifferences>::iterator i = o2iMetaCache.begin();
            i!= o2iMetaCache.end();
            i++)
        delete i->second;*/
}

BoundVertex Rule::compute(const vector<BoundVertex>& h, pHandle CX,
                          bool fresh) const
{
    bindingsT h_b;
    foreach(const BoundVertex& bv, h)
    {
        if (asw->isType(_v2h(bv.value)))
        {
            puts("!isReal");
            printf("%u\n", _v2h(bv.value)); 
            NMPrinter(NMP_ALL)(_v2h(bv.value), -10);
        }

        assert(!asw->isType(_v2h(bv.value)));
        assert(asw->getType(_v2h(bv.value)) != FW_VARIABLE_NODE);
    }
    
    vector<Vertex> rule_args(h.size());
    transform(h.begin(), h.end(), rule_args.begin(), DropVertexBindings());

    Btr<bindingsT> bindings_of_all_args(new bindingsT);

    for(vector<BoundVertex>::const_iterator bv = h.begin(); bv != h.end(); bv++) {
        if (bv->bindings) {
            insert_with_consistency_check(*bindings_of_all_args,
                                          bv->bindings->begin(),
                                          bv->bindings->end());
            
            cprintf(3,"Bind [%d]:\n", _v2h(bv->value));
            
            foreach(hpair phh, *bv->bindings)
            {
                printTree(phh.first,0,3);
                cprintf(3,"=>");
                printTree(phh.second,0,3);
            }
        }
    }

    return BoundVertex(compute(rule_args, CX, fresh), bindings_of_all_args);
}

bool Rule::validate(const vector<Vertex>& h) const
{
    if (freeInputArity)
        return true;

    const uint n = h.size();
    
    vector<Vertex> myh(h);
    
    if (n != inputFilter.size()) {
        cprintf(0,"Rule::validate FALSE. Input vector size: %d\n", n);
#if 0        
        for (int i=0;i<n;i++)
            printTree(v2h(h[i]),0,0);
#else 
        //NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
        //  bind(NMPrinter(), bind(&_v2h, _1))(h[0]);
    //      NMPrinter()(bind(v2h(h[0]));
    //  for_each(h.begin(), h.end(), NMPrinter(NMP_HANDLE|NMP_TYPE_NAME));
    //        for (int i=0;i<n;i++)
    //   printer.print(v2h(h[i]));
#endif             
        return false;
    }
    
    // A vertex wrapper contains a vtree in a weak_atom
    //typedef weak_atom<boost::shared_ptr<tree<Vertex> > > vertex_wrapper;
    typedef weak_atom< meta > vertex_wrapper;
    // Check and sort the vector of inputs so that they satisfy the
    // input filter...
    for (uint i = 0; i < n; i++) {
        vertex_wrapper mp(inputFilter[i]);
        pHandle h2 = _v2h(myh[i]);
        if (!(mp(h2)))
        {
            uint r=0;
            for ( r = i+1;
                  r < n && !mp(_v2h(myh[r]));
                  r++);
            if (r < n) {
                std::swap(myh[i], myh[r]);
            } else {
                cprintf(0,"Rule::validate FALSE. Input vector:\n");
                NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
                for (uint j=0;j<n;j++)
                    printer.print(_v2h(h[j]));
                return false;
            }
        }
    }
    return true;
}

BoundVertex Rule::computeIfValid(const vector<Vertex>& h, pHandle CX,
                                 bool fresh) const
{
    if (validate(h))
        return compute(h,CX, fresh);
    else
        return Vertex((pHandle) PHANDLE_UNDEFINED);
}

Rule::setOfMPs Rule::o2iMeta(meta outh) const
{
    assert(composer);
    setOfMPs ret;
    
    const int N = outh->begin().number_of_children();
    if (N < 1)
        return ret;
/*  if (Handle* ph = v2h(*outh.begin()))
        return makeSingletonSet(vector2<meta>(
        //Note: vector2 has been removed because list_of can be used instead
        new tree<Vertex>()));*/
    bool overrideInputFilter = false;
    setOfMPs extraFilter = o2iMetaExtra(outh, overrideInputFilter);
    if (extraFilter.empty())
        return setOfMPs();

    /// Ensure that there're no Handles involved! (Or, it's a special unprovable type,
    /// in which case there may be a hack involved.
    /* set<boost::shared_ptr<MPs> >::iterator e;
    for (e = extraFilter.begin(); e!=extraFilter.end(); e++) {
        for (Rule::MPs::iterator m = (*e)->begin(); m != (*e)->end(); m++) {
            puts(":::");
            assert(!(*m)->real || UnprovableType((*m)->T));
        }
    }*/
    
    if (overrideInputFilter)
        return extraFilter;
/** @todo Update this to the new types system
    for (e = extraFilter->begin(); e!=extraFilter.end(); e++)
    {
        assert((*e)->size() == inputFilter.size());
        boost::shared_ptr<MPs> ret1(new MPs);
        
        for (uint i = 0; i < inputFilter.size(); i++)
            ret1->push_back(boost::shared_ptr<atom>(
                new atom(AND_LINK, 2,
                    new atom(*inputFilter[i]),
                    new atom(*(**e)[i])
                )
            ));
        
        ret->insert(ret1);
    }*/
    
    return ret;
}

Rule::setOfMPs Rule::fullInputFilter() const
{
    meta genericTarget(targetTemplate());
    
    if (genericTarget) {
        return o2iMeta(genericTarget);
    } else {
        // convert the inputFilter member to BoundVertex instances.
        
        //Btr<std::vector<BBvtree> > filter(new std::vector<BBvtree>);
        std::vector<BBvtree> filter;
        
        foreach(meta item, inputFilter) {
                        
            BBvtree Btr_bound_item(new BoundVTree(*item)); // not bound yet, it just means it can be bound
            filter.push_back(Btr_bound_item);
        }
        
        //return makeSingletonSet(filter);
        
        Rule::setOfMPs ret;
        ret.insert(filter);
        return ret;
    }
}

void Rule::CloneArgs(const Rule::MPs& src, Rule::MPs& dest)
{
    foreach(const BBvtree& bbvt, src)
        dest.push_back(Btr<BoundVTree>(bbvt->Clone()));
}

}} //namespace opencog { namespace pln {
