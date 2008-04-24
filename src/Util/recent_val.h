#ifndef _UTIL_RECENT_VAL_H
#define _UTIL_RECENT_VAL_H

/* Author Joel Pitt April 2008
 * Copyright SIAI 2008
 */

namespace Util {
    /**
     * recent_val is a value that can update which also
     * keeps a exponential decaying record of it's recent value
     */
    template<class ValueType> struct recent_val {
	/** The current value */
	ValueType val;
	/** The recent record of the value */
	float recent;
	/** The decay rate of the recent value */
	float decay;

	recent_val(ValueType x): val(x), recent((float)x), decay(0.5f) {};
	recent_val(): val(0), recent(0.0f), decay(0.5f) {};
	
	/** call this method to update the current value and the decaying
	 * record too.
	 * x the new value
	 */
	inline void update(ValueType x) { val = x;
	    recent = ((decay) * val) + ((1.0f - decay) * recent); };

    };
}

#endif // _UTIL_RECENT_VAL_H
