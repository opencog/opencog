#ifndef _OPENCOG_FORGETTING_AGENT_WRAP_H
#define _OPENCOG_FORGETTING_AGENT_WRAP_H

/** Exposes the ForgettingAgent class. */
void init_ForgettingAgent_py();

/** A class wrapper of the ForgettingAgent class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
/*struct ForgettingAgentWrap : ForgettingAgent, wrapper<ForgettingAgent>
{
    // Pure virtual functions.

    strength_t getMean() const;
    count_t getCount() const;
    confidence_t getConfidence() const;

    float toFloat() const;
    std::string toString() const;
    ForgettingAgentType getType() const;

    ForgettingAgent* clone() const;

    ForgettingAgent& operator=(const ForgettingAgent&);
    bool operator==(const ForgettingAgent&) const;

    // Non-pure virtual functions.

    ForgettingAgent* merge(const ForgettingAgent&) const;
    ForgettingAgent* default_merge(const ForgettingAgent&) const;
    bool isNullTv() const;
    bool default_isNullTv() const;
};*/

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
//static ForgettingAgent* (*factoryx1)(const char*) = &ForgettingAgent::factory;

/** A function pointer necessary for wrapping overloaded member methods of 
 * classes.  */
//static ForgettingAgent* (*factoryx2)(ForgettingAgentType, const char*) = &ForgettingAgent::factory;

#endif // _OPENCOG_FORGETTING_AGENT_WRAP_H
