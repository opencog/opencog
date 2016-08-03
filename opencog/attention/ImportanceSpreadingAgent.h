/*
 * opencog/attention/ImportanceSpreadingAgent.h
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

#ifndef _OPENCOG_IMPORTANCE_SPREADING_AGENT_H
#define _OPENCOG_IMPORTANCE_SPREADING_AGENT_H

#include <string>

#include <math.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/cogserver/server/Agent.h>
#include <opencog/util/Logger.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

/** Spreads short term importance along HebbianLinks.
 *
 * Currently only spreads along Symmetric and Inverse HebbianLinks.
 *
 * @todo Spread along asymmetric hebbian links too.
 * @todo Spread along symmetric inverse hebbian links too.
 * @todo Optionally spread long term importance.
 */
class ImportanceSpreadingAgent : public Agent
{
private:
    //! Minimal amount of STI necessary for an atom to have before it spreads
    //! STI.
    AttentionValue::sti_t spreadThreshold;

    //! How much to multiply the HebbianLink TruthValue to convert to STI.
    double importanceSpreadingMultiplier;

    //! Whether to spread STI across all types of Links and not just HebbianLinks
    bool allLinksSpread;

    //! The 
    AttentionValue::sti_t stealingLimit;

    /** Spread importance for an atom.
     *
     * @param h The handle for the atom to spread importance for.
     */
    void spreadAtomImportance(Handle h);

    //! Total amount spread during recent runs.
    opencog::recent_val<long> amountSpread;

    //! Spread importance along Hebbian links.
    void spreadImportance();

    //! Sum total difference for an atom
    int sumTotalDifference(Handle source, HandleSeq& links);

    //! Sum difference for one link
    int sumDifference(Handle source, Handle link);

    //! Calculate the difference for an inverse link
    double calcInverseDifference(AttentionValue::sti_t s, AttentionValue::sti_t t, \
            double weight);

    //! Calculate the difference for a normal Hebbian link
    double calcDifference(AttentionValue::sti_t s, AttentionValue::sti_t t, \
            double weight);

public:

    virtual const ClassInfo& classinfo() const { return info(); }
    static const ClassInfo& info() {
        static const ClassInfo _ci("opencog::ImportanceSpreadingAgent");
        return _ci;
    }

    ImportanceSpreadingAgent(CogServer&);
    virtual ~ImportanceSpreadingAgent();
    virtual void run();

    /** Set minimal amount of STI necessary for an atom to have before it
     * spreads STI.
     *
     * @param t the threshold.
     */
    void setSpreadThreshold(AttentionValue::sti_t t) {
        spreadThreshold = t;
    };

    /** Set the multiplier for converting the HebbianLink TruthValue to STI.
     *
     * If multiplier is zero, then \b all STI above the threshold is evenly
     * distributed.
     *
     * @param m the multiplier.
     */
    void setImportanceSpreadingMultiplier(double m) {
        importanceSpreadingMultiplier = m;
    };

    class IsHebbianLink {
        public:
        AtomSpace* a;
        IsHebbianLink(AtomSpace* _a): a(_a) {};
        
        bool operator()(Handle h);
    };
}; // class

typedef std::shared_ptr<ImportanceSpreadingAgent> ImportanceSpreadingAgentPtr;

/** @}*/
} // namespace opencog

#endif // _OPENCOG_IMPORTANCE_SPREADING_AGENT_H
