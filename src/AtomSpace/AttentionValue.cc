// Copyright(c)	2007 Novamente LLC  All rights reserved.
// Filename:	$(src)\core\AttentionValue.cpp
// Author:		Tony Lofthouse	
// Created:		24 Apr 2007
// Updates:

#include "AttentionValue.h"

AttentionValue::AttentionValue(sti_t STI, lti_t LTI, vlti_t VLTI) {
	m_STI = STI;
	m_LTI = LTI;
	m_VLTI = VLTI;
}

AttentionValue::sti_t AttentionValue::getSTI() const {
	return m_STI;
}

float AttentionValue::getScaledSTI() const {
	return (((float) m_STI) + 32768) / 65534;
}

AttentionValue::lti_t AttentionValue::getLTI() const {
	return m_LTI;
}
 
AttentionValue::vlti_t AttentionValue::getVLTI() const {
	return m_VLTI;
}

void AttentionValue::decaySTI() {
	if (m_STI != -32768)
		m_STI--;
}

std::string AttentionValue::toString() const {
	char buffer[256];
	sprintf(buffer, "[%d, %d, %s]", (int)m_STI, (int)m_LTI, m_VLTI ? "NONDISPOSABLE" : "DISPOSABLE");
	return buffer;
}

AttentionValue* AttentionValue::clone() const {
	return new AttentionValue(m_STI, m_LTI, m_VLTI);
}

bool AttentionValue::operator==(const AttentionValue& av) const {
	return (m_STI == av.getSTI() && m_LTI == av.getLTI() && m_VLTI == av.getVLTI());
}


AttentionValue* AttentionValue::m_defaultAV = NULL;

const AttentionValue& AttentionValue::getDefaultAV() {
	if(!m_defaultAV)
		m_defaultAV = AttentionValue::factory();
	return *m_defaultAV;
}

AttentionValue* AttentionValue::factory() {
	return new AttentionValue(DEFAULTATOMSTI, DEFAULTATOMLTI, DEFAULTATOMVLTI);
}

AttentionValue* AttentionValue::factory(AttentionValue::sti_t sti) {
	return new AttentionValue(sti, DEFAULTATOMLTI, DEFAULTATOMVLTI);
}

AttentionValue* AttentionValue::factory(float scaledSti) {

    AttentionValue::sti_t sti;

    if (scaledSti >= 1.0) {
        sti = 32767;
    } else if (scaledSti <= 0.0) {
        sti = -32768;
    } else {
        sti = (AttentionValue::sti_t) ((65534 * scaledSti) - 32767);
    }

	return new AttentionValue(sti, DEFAULTATOMLTI, DEFAULTATOMVLTI);
}

AttentionValue* AttentionValue::factory(AttentionValue::sti_t sti, AttentionValue::lti_t lti) {
	return new AttentionValue(sti, lti, DEFAULTATOMVLTI);
}

AttentionValue* AttentionValue::factory(AttentionValue::sti_t sti, AttentionValue::lti_t lti, AttentionValue::vlti_t vlti) {
	return new AttentionValue(sti, lti, vlti);
}

