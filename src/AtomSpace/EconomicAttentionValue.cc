#include "EconomicAttentionValue.h"

EconomicAttentionValue::stim_t EconomicAttentionValue::stimulate(EconomicAttentionValue::stim_t amount) 
{
    // always greater than zero as stim_t is unsigned
    // if (amount >= 0)
    stimulus += amount;
    return stimulus;
}
	
EconomicAttentionValue* EconomicAttentionValue::factory()
{
    return new EconomicAttentionValue(DEFAULTATOMSTI, DEFAULTATOMLTI, DEFAULTATOMVLTI);
}

std::string EconomicAttentionValue::toString() const {
    char buffer[256];
    sprintf(buffer, "[%d, %d, %s, %d]", (int) getSTI(), (int) getLTI(), getVLTI() ? "NONDISPOSABLE" : "DISPOSABLE", (int) getStimulus() );
    return buffer;
}

EconomicAttentionValue* EconomicAttentionValue::clone() const
{
    EconomicAttentionValue *eav;

    eav = new EconomicAttentionValue(getSTI(), getLTI(), getVLTI());
    eav->stimulate(getStimulus());
    return eav;
}

bool EconomicAttentionValue::operator==(const EconomicAttentionValue& av) const {
    return (getSTI() == av.getSTI() && getLTI() == av.getLTI()
	    && getVLTI() == av.getVLTI() && getStimulus() == av.getStimulus());
}

EconomicAttentionValue* EconomicAttentionValue::m_defaultAV = NULL;

const EconomicAttentionValue& EconomicAttentionValue::getDefaultAV() {
    if(!m_defaultAV)
	m_defaultAV = EconomicAttentionValue::factory();
    return *m_defaultAV;
}


EconomicAttentionValue* EconomicAttentionValue::factory(sti_t sti)
{
    return new EconomicAttentionValue(sti, DEFAULTATOMLTI, DEFAULTATOMVLTI);
}

EconomicAttentionValue* EconomicAttentionValue::factory(float scaledSti)
{
    AttentionValue::sti_t sti;

    if (scaledSti >= 1.0) {
        sti = 32767;
    } else if (scaledSti <= 0.0) {
        sti = -32768;
    } else {
        sti = (AttentionValue::sti_t) ((65534 * scaledSti) - 32767);
    }

    return new EconomicAttentionValue(sti, DEFAULTATOMLTI, DEFAULTATOMVLTI);
}

EconomicAttentionValue* EconomicAttentionValue::factory(sti_t sti, lti_t lti)
{
    return new EconomicAttentionValue(sti, lti, DEFAULTATOMVLTI);
}

EconomicAttentionValue* EconomicAttentionValue::factory(sti_t sti, lti_t lti, vlti_t vlti)
{
    return new EconomicAttentionValue(sti, lti, vlti);
}
