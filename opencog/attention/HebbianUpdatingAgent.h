/*
 * opencog/attention/HebbianUpdatingAgent.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#ifndef _OPENCOG_HEBBIAN_LEARNING_AGENT_H
#define _OPENCOG_HEBBIAN_LEARNING_AGENT_H

#include <string>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/Agent.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

/** Agent that carries out simple Hebbian learning.
 *
 * @note Only updates existing HebbianLinks.
 * @todo Support SymmetricInverseHebbianLinks and AsymmetricHebbianLinks
 * @todo Support Hebbian links with arity > 2
 */
class HebbianUpdatingAgent : public Agent
{

protected:
    AtomSpace* a;

	/** Work out the conjunction between a series of handles.
	 *
	 * The returned value is the correlation between distance in/out of
	 * the attentional focus. STI of atoms are normalised (separately
	 * for atoms within the attentional focus and those below it) and then
	 * multiplied. Only returns a non zero value if at least one atom
	 * is in the attentional focus.
	 *
	 * @param handles A vector of handles to calculate the conjunction for.
	 * @return conjunction between -1 and 1.
	 * @todo create a method for working out conjunction between more than
	 * two atoms.
	 */
    float targetConjunction(HandleSeq handles);

	/** Transform STI into a normalised STI value between -1 and 1.
	 *
	 * @param s STI to normalise.
	 * @return the normalised STI between -1.0 and 1.0
	 */
    float getNormSTI(AttentionValue::sti_t s);

	/** Rearrange the the vector so that the one with a positive normalised
	 * STI is at the front.
	 *
	 * @param outgoing Vector to rearrange.
	 * @return rearranged vector.
	 */
    HandleSeq& moveSourceToFront(HandleSeq &outgoing);

    void setMean(Handle h, float tc);

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::HebbianUpdatingAgent");
        return _ci;
    }

    HebbianUpdatingAgent(CogServer&);
    virtual ~HebbianUpdatingAgent();
    virtual void run();

    //! Whether to convert links to/from InverseHebbianLinks as necessary.
    bool convertLinks;

    //! Maximum allowable LTI of a link to be converted.
    AttentionValue::lti_t conversionThreshold;

	/** Update the TruthValues of the HebbianLinks in the AtomSpace.
     *
     * @todo Make link strength decay parameter configurable. Currently
     * hard-code, ugh.
     */
    void hebbianUpdatingUpdate();

}; // class

typedef std::shared_ptr<HebbianUpdatingAgent> HebbianUpdatingAgentPtr;

/** @}*/
} // namespace

#endif // _OPENCOG_HEBBIAN_LEARNING_AGENT_H
