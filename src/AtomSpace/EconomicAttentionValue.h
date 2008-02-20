#ifndef ECONOMICATTENTIONVALUE_H_
#define ECONOMICATTENTIONVALUE_H_

#include "AttentionValue.h"

class EconomicAttentionValue : public AttentionValue
{
public:
    typedef unsigned short stim_t;
private:
    stim_t stimulus;

    static EconomicAttentionValue* m_defaultAV;
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
    stim_t getStimulus() const { return stimulus; }
    
    /**
     * Reset the stimulus counter.
     * 
     * @returns the new stimulus value, which should be 0.
     */
    stim_t resetStimulus() { stimulus = 0; return stimulus; }

    virtual bool operator==(const EconomicAttentionValue& av) const;

    // Returns const string "[sti_val, lti_val, vlti_val, stim_val]"
    // @param none
    virtual std::string toString() const;

    /**
* Returns a copy of the EconomicAttentionValue, stimulus in new copy is equal to zero.
*
*/
    virtual EconomicAttentionValue* clone() const;
    
    // STATIC METHODS

    // Returns a shared EconomicAttentionValue with default STI, LTI, VLTI values
    // @param none
    static const EconomicAttentionValue& getDefaultAV();
    // factory methods
    static EconomicAttentionValue* factory();
    static EconomicAttentionValue* factory(sti_t sti);
    static EconomicAttentionValue* factory(float scaledSti);
    static EconomicAttentionValue* factory(sti_t sti, lti_t lti);
    static EconomicAttentionValue* factory(sti_t sti, lti_t lti, vlti_t vlti);
};

#endif /*ECONOMICATTENTIONVALUE_H_*/
