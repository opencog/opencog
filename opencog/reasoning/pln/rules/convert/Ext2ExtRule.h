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

#ifndef EXT2EXTRULE_H
#define EXT2EXTRULE_H

namespace opencog { namespace pln {

template<Type IN_LINK_TYPE, Type OUT_LINK_TYPE>
class Ext2ExtRule : public Rule
{
public:
    Ext2Ext()
    {
        inputFilter.push_back(Btr<atom>(new atom(INSTANCE_OF, 1, new atom(IN_LINK_TYPE))));
    }
    virtual atom i2oType(Handle* h, const int n) const
    {
        return atom(OUT_LINK_TYPE, 2,
                    new atom(CONCEPT_NODE,""),
                    new atom(CONCEPT_NODE,""));
        
    }
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        if (!inheritsType(outh.T, OUT_LINK_TYPE))
            return Rule::setOfMPs();
        Btr<MPs> ret(new MPs);
        ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, IN_LINK_TYPE)));
        return ret;
    }
    
    virtual bool valid(Handle* h, const int n) const
    {
        assert(1==n);
        
        return isSubType(h[0], IN_LINK_TYPE);
    }
    BoundVertex compute(const VertexSeq& premiseArray, Handle CX = NULL,
                        bool fresh = true) const
    {
        assert(n == 1);
        LINKTYPE_ASSERT(premiseArray[0], IN_LINK_TYPE);
        
        std::vector<Handle> in_args = asw->getOutgoing(premiseArray[0]);
        
        TruthValuePtr retTV(asw->getTV(premiseArray[0]));
        
        std::vector<Handle> out_args;
        out_args.push_back(SatisfyingSet(in_args[0]));
        out_args.push_back(SatisfyingSet(in_args[1]));
	
        Handle p = asw->addLink(OUT_LINK_TYPE, out_args, retTV, fresh);
        
        return ret;
    }
};

typedef Ext2ExtRule<EXTENSIONAL_IMPLICATION_LINK, SUBSET_LINK> ExtImpl2SubsetRule;
typedef Ext2ExtRule<EXTENSIONAL_EQUIVALENCE_LINK, EXTENSIONAL_SIMILARITY_LINK> ExtEqui2ExtSimRule;
typedef Ext2ExtRule<EQUIVALENCE_LINK, SIMILARITY_LINK> Equi2SimRule;

}} // namespace opencog { namespace pln {
#endif // EXT2EXTRULE_H
