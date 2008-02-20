#ifndef ECONOMICATTENTIONVALUE_H_
#define ECONOMICATTENTIONVALUE_H_

#include "AttentionValue.h"

class EconomicAttentionValue : public AttentionValue
{
public:
	typedef short stim_t;
private:
	stim_t stimulus;
	
public:

	/**
	 * CLASS CONSTRUCTOR	
	 *
	 * @param int STI: The STI value to set for the atom
	 * @param int LTI: The LTI value to set for the atom
	 * @param unsigned short VLTI: The VLTI flag value to set for this atom
	 */
	EconomicAttentionValue(sti_t STI, lti_t LTI, vlti_t VLTI) : AttentionValue (STI, LTI, VLTI) { stimulus = 0; };
	virtual ~EconomicAttentionValue() {};
	
	/**
	 * Provide stimulus to the attention value.
	 * 
	 * @param amount - the amount of stimulus to pass.
	 * @returns the current total stimulus associated with this attention value.  
	 */ 
	stim_t stimulate(stim_t amount);
	
	/**
	 * Get total stimulus associated with this attention value.
	 *
	 * @returns the current total stimulus associated with this attention value.
	 */
	stim_t getStimulus() { return stimulus; }
	
	/**
	 * Reset the stimulus counter.
	 * 
	 * @returns the new stimulus value, which should be 0.
	 */
	stim_t resetStimulus() { stimulus = 0; return stimulus; }
	
	// factory methods
	static EconomicAttentionValue* factory();
	static EconomicAttentionValue* factory(sti_t sti);
	static EconomicAttentionValue* factory(float scaledSti);
	static EconomicAttentionValue* factory(sti_t sti, lti_t lti);
	static EconomicAttentionValue* factory(sti_t sti, lti_t lti, vlti_t vlti);
};

#endif /*ECONOMICATTENTIONVALUE_H_*/
