#include <opencog/attentionbank/avalue/AttentionValue.h>
#include <opencog/atomspace/AtomSpace.h>

#ifndef ATTENTION_STAT_H
#define ATTENTION_STAT_H
/**
 * Attention Related statistics.
 * Things we want to know:
 *    - What time did an an atom leave the AF.
 *    - How much sti did an atom gained via spreading.
 *      - How much of the spreading happened via Hebbian links.
 *      - How much of the spreading happend via non-hebbian links.
 *    - How much sti did an atom gained via stimulus.
 *      - How often has it been stimulated?
 *    - How much rent did an atom pay.
 *       - How often has been charged?
 */

using av_sti = opencog::AttentionValue::sti_t;

struct AVStat{
    av_sti heblink_sti_gain = 0;
    av_sti link_sti_gain = 0;
    av_sti direct_sti_gain = 0; 
    av_sti spreading = 0;  
    av_sti rent = 0;
};

extern std::unordered_map<opencog::Handle, AVStat> atom_avstat;

 /*  
 *  TODO
 *  Derive direct_sti_gain from the above two
 *
 */
#endif
