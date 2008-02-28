/**
 * AttentionValue.h
 *
 * Copyright(c)	2007 Novamente LLC  All rights reserved.
 * Author:		Tony Lofthouse	
 * Created:		24 Apr 2007
 */

#pragma once
#ifndef ATTENTIONVALUE_H
#define ATTENTIONVALUE_H

#include <types.h>
#include <limits.h>
#include <string>

struct AttentionValue {
public:
	typedef short sti_t;			// short-term importance type
	typedef short lti_t;			// long-term importance type
	typedef unsigned short vlti_t;	// very long-term importance type

	static const int DISPOSABLE = 0;	//Status flag for vlti 
	static const int NONDISPOSABLE = 1; //Status flag for vlti 

	// CLASS CONSTANTS
	static const sti_t DEFAULTATOMSTI = 500;
	static const lti_t DEFAULTATOMLTI = 0;
	static const vlti_t DEFAULTATOMVLTI = DISPOSABLE;

	static const sti_t MAXSTI = SHRT_MAX;
	static const lti_t MAXLTI = SHRT_MAX;


	private:

	//CLASS FIELDS
	sti_t m_STI;
	lti_t m_LTI;
	vlti_t m_VLTI;	//Needs to be chnaged to a bit field after debugging
	static AttentionValue* m_defaultAV;

	public:	
	virtual ~AttentionValue() {}
	// CLASS CONSTRUCTORS	

	// @param int STI: The STI value to set for the atom
	// @param int LTI: The LTI value to set for the atom
	// @param unsigned short VLTI: The VLTI flag value to set for this atom
	AttentionValue(sti_t STI, lti_t LTI, vlti_t VLTI);

	// PUBLIC GET/SET PROPERTIES

	// return STI property value
	virtual sti_t getSTI() const;
	virtual float getScaledSTI() const;

	// return LTI property value
	virtual lti_t getLTI() const;

	// return VLTI property value
	virtual vlti_t getVLTI() const;

	// PUBLIC METHODS

	// Decays short term importance
	void  decaySTI();

	// Returns const string "[sti_val, lti_val, vlti_val]"
	// @param none
	virtual std::string toString() const;

	// Returns An AttentionValue* cloned from this AttentionValue
	// @param none
	virtual AttentionValue* clone() const;

	// Compares two AttentionValues and returns true if the 
	// elements are equal false otherwise
	// @param none
	virtual bool operator==(const AttentionValue& av) const;

	// STATIC METHODS

	// Returns a shared AttentionValue with default STI, LTI, VLTI values
	// @param none
	static const AttentionValue& getDefaultAV();

	// factory methods
	static AttentionValue* factory();
	static AttentionValue* factory(sti_t sti);
	static AttentionValue* factory(float scaledSti);
	static AttentionValue* factory(sti_t sti, lti_t lti);
	static AttentionValue* factory(sti_t sti, lti_t lti, vlti_t vlti);
};
#endif
