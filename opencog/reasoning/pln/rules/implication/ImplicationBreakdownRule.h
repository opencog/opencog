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

#ifndef IMPBREAKDOWNRULE_H
#define IMPBREAKDOWNRULE_H

namespace opencog { namespace pln {

/** (A->B) => B. */
class ImplicationBreakdownRule : public Rule
{
public:
    NO_DIRECT_PRODUCTION;
    
    ImplicationBreakdownRule(AtomSpaceWrapper *_asw);
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
    
    /**
     * Similar to StrictImplicationBreakdownRule but does not
     * explicitly uses A in the premises.
     * A is still used in the computation but it is fetched by selecting
     * a first outgoing of A->B
     *
     * @param premiseArray vector of premises, here the sole element (A->B)
     * @param CX Context to use for rule computation. Currently unused.
     *
     * @return the BoundVertex of the conclusion, A
     */
    BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const;
    bool validate2(MPs& args) const { return true; }
};

}} // namespace opencog { namespace pln {
#endif // IMPBREAKDOWNRULE_H
