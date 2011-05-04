/** AverageInstantiationRule.h --- 
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


#ifndef _OPENCOG_AVERAGEINSTANTIATIONRULE_H
#define _OPENCOG_AVERAGEINSTANTIATIONRULE_H

namespace opencog { namespace pln {

static const std::string AverageInstantiationRulePrefixStr = 
    "AverageInstantiationRule";

/**
 * Rule to instantiate the body of an AverageLink. That is
 *
 * AverageLink <TV>
 *     ListLink
 *         X1
 *         ...
 *         Xn
 *     Body(X1, ..., Xn)
 * |-
 * Body(X1/A1, ..., Xn/An) <TV>
 *
 * that is, under no additional assumption the TV of the instance is
 * the TV of the AverageLink, same strength and confidence.
 *
 * Of course you must assume that the confidence of the AverageLink
 * has been correctly calculated in the first place.
 *
 * It should be right because Body(A).TV.s is exactly distributed
 * according to TV itself.
 */
class AverageInstantiationRule : public BaseInstantiationRule<IdentityFormula> {
private:
    typedef BaseInstantiationRule<IdentityFormula> super;

public:
    //! Constructor for calling from add/remove signal handler
    AverageInstantiationRule(Type t, pHandle _averageLink, AtomSpaceWrapper *_asw) :
        super(t, _averageLink, _asw, AverageInstantiationRulePrefixStr) { }

    AverageInstantiationRule(pHandle _averageLink, AtomSpaceWrapper *_asw) :
        super(_averageLink, _asw, AverageInstantiationRulePrefixStr) { }
};

}} // namespace opencog { namespace pln {

#endif // _OPENCOG_AVERAGEINSTANTIATIONRULE_H
