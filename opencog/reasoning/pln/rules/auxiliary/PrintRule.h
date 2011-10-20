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

#ifndef PRINTRULE_H
#define PRINTRULE_H

namespace opencog { namespace pln {

class PrintRule : public Rule
{
public:
    PrintRule(AtomSpaceWrapper *_asw)
	: Rule(_asw,false,true,"PrintRule")
    {
        inputFilter.push_back(meta(
                                   new tree<Vertex>(mva((Handle)ATOM))));
        inputFilter.push_back(meta(
                                   new tree<Vertex>(mva((Handle)ATOM))));
    }
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        return Rule::setOfMPs();
    }
    
    NO_DIRECT_PRODUCTION;

    BoundVertex compute(const VertexSeq& premiseArray,
                        Handle CX = NULL,
                        bool fresh = true) const
    {
        for (int i = 0; i < premiseArray.size(); i++)
            printTree(premiseArray[i], 0, 0);
    }
};
#endif

}} // namespace opencog { namespace pln {
#endif // PRINTRULE_H
