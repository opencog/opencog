#include "EconomicAttentionValue.h"

EconomicAttentionValue::stim_t EconomicAttentionValue::stimulate(EconomicAttentionValue::stim_t amount) 
{
	stimulus += amount;
	return stimulus;
}
	
EconomicAttentionValue* EconomicAttentionValue::factory()
{
	return new EconomicAttentionValue(DEFAULTATOMSTI, DEFAULTATOMLTI, DEFAULTATOMVLTI);
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
