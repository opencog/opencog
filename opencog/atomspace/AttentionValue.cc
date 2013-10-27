/*
 * opencog/atomspace/AttentionValue.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Tony Lofthouse <tony_lofthouse@btinternet.com>
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

#include "AttentionValue.h"

#include <math.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/util/platform.h>

using namespace opencog;

const AttentionValue::sti_t AttentionValue::DEFAULTATOMSTI = 0;
const AttentionValue::lti_t AttentionValue::DEFAULTATOMLTI = 0;
const AttentionValue::vlti_t AttentionValue::DEFAULTATOMVLTI = 0;

AttentionValue::AttentionValue(sti_t STI, lti_t LTI, vlti_t VLTI)
{
    m_STI = STI;
    m_LTI = LTI;
    m_VLTI = VLTI;
}

AttentionValue::sti_t AttentionValue::getSTI() const
{
    return m_STI;
}

float AttentionValue::getScaledSTI() const
{
    return (((float) m_STI) + 32768) / 65534;
}

AttentionValue::lti_t AttentionValue::getLTI() const
{
    return m_LTI;
}

AttentionValue::vlti_t AttentionValue::getVLTI() const
{
    return m_VLTI;
}

void AttentionValue::decaySTI()
{
    // Prevent m_STI from wrapping around... should really compare to system
    // constant though.
    if (m_STI != -32768) m_STI--;
}

std::string AttentionValue::toString() const
{
    char buffer[256];
    sprintf(buffer, "[%d, %d, %s]", (int)m_STI, (int)m_LTI, m_VLTI ? "NONDISPOSABLE" : "DISPOSABLE");
    return buffer;
}

AttentionValuePtr AttentionValue::clone() const
{
    return createAV(m_STI, m_LTI, m_VLTI);
}

AttentionValue* AttentionValue::rawclone() const
{
    return new AttentionValue(m_STI, m_LTI, m_VLTI);
}

bool AttentionValue::operator==(const AttentionValue& av) const
{
    return (m_STI == av.getSTI() && m_LTI == av.getLTI() && m_VLTI == av.getVLTI());
}


AttentionValuePtr AttentionValue::m_defaultAV = NULL;

AttentionValuePtr AttentionValue::getDefaultAV()
{
    if (!m_defaultAV)
        m_defaultAV = createAV();
    return m_defaultAV;
}

bool AttentionValue::STISort::test(AtomPtr h1, AtomPtr h2)
{
    return h1->getAttentionValue()->getSTI() >
           h2->getAttentionValue()->getSTI();
}

bool AttentionValue::LTIAndTVAscendingSort::test(AtomPtr h1, AtomPtr h2)
{
    lti_t lti1, lti2;
    float tv1, tv2;

    tv1 = fabs(h1->getTruthValue()->getMean());
    tv2 = fabs(h2->getTruthValue()->getMean());

    lti1 = h1->getAttentionValue()->getLTI();
    lti2 = h2->getAttentionValue()->getLTI();

    if (lti1 < 0)
        tv1 = lti1 * (1.0f - tv1);
    else
        tv1 = lti1 * tv1;

    if (lti2 < 0)
        tv2 = lti2 * (1.0f - tv2);
    else
        tv2 = lti2 * tv2;

    return tv1 < tv2;
}

bool AttentionValue::LTIThenTVAscendingSort::test(AtomPtr h1, AtomPtr h2)
{
    lti_t lti1, lti2;
    lti1 = h1->getAttentionValue()->getLTI();
    lti2 = h2->getAttentionValue()->getLTI();

    if (lti1 != lti2) return lti1 < lti2;

    float tv1, tv2;
    tv1 = h1->getTruthValue()->getMean();
    tv2 = h2->getTruthValue()->getMean();
    return tv1 < tv2;
}
