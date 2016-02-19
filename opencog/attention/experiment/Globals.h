/*
 * Globals.h
 *
 *  Created on: 15 Feb 2016
 *      Author: misgana
 */

#ifndef OPENCOG_ATTENTION_EXPERIMENT_GLOBALS_H_
#define OPENCOG_ATTENTION_EXPERIMENT_GLOBALS_H_

#include <opencog/cogserver/server/Agent.h>
#include <opencog/cogserver/server/Factory.h>

namespace opencog {
namespace ECANExperiment {
struct AValues {
    AttentionValue::sti_t _sti;
    AttentionValue::lti_t _lti;
    AttentionValue::vlti_t _vlti;
    long _cycle;

    AValues() :
            _sti(0), _lti(0), _vlti(0), _cycle(0)
    {
    }

    AValues(AttentionValue::sti_t sti, AttentionValue::lti_t lti,
            AttentionValue::vlti_t vlti, long cycle) :
            _sti(sti), _lti(lti), _vlti(vlti), _cycle(cycle)
    {
    }
};

struct TValues {
    strength_t _strength;
    confidence_t _confidence;
    long _cycle;

    TValues() :
            _strength(0.0), _confidence(0.0), _cycle(0)
    {
    }

    TValues(strength_t strength, confidence_t confidence, long cycle) :
            _strength(strength), _confidence(confidence), _cycle(cycle)
    {
    }
};

extern std::vector<std::string> generated_sentences;
extern std::vector<HandleSeq> sent_wordnodes;
extern std::vector<HandleSeq> wordinstancenodes;

extern UnorderedHandleSet hspecial_word_nodes;

extern std::vector<std::string> special_words;
extern std::vector<std::string> nspecial_words;
extern int sent_size;

extern int special_word_occurence_period;

bool are_similar(const Handle& h1, const Handle& h2, bool strict_type_match);
bool exists_in(const Handle& hlink, const Handle& h);

bool is_smokes_reln(const Handle& h);
bool is_friendship_reln(const Handle& h);
bool is_cancer_reln(const Handle& h);

void save(const std::string& filename, const HandleSeq& seq,
          const std::string& header);

}
}

#endif /* OPENCOG_ATTENTION_EXPERIMENT_GLOBALS_H_ */
