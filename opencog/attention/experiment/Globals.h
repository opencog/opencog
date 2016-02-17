/*
 * Globals.h
 *
 *  Created on: 15 Feb 2016
 *      Author: misgana
 */

#ifndef OPENCOG_ATTENTION_EXPERIMENT_GLOBALS_H_
#define OPENCOG_ATTENTION_EXPERIMENT_GLOBALS_H_

namespace opencog {
namespace ECANExperiment {
std::vector<std::string> generated_sentences;
std::vector<HandleSeq> sent_wordnodes;
std::vector<HandleSeq> wordinstancenodes;

UnorderedHandleSet hspecial_word_nodes;

std::vector<std::string> special_words;
std::vector<std::string> nspecial_words;
int sent_size;

int special_word_occurence_period = 2;
}
}

#endif /* OPENCOG_ATTENTION_EXPERIMENT_GLOBALS_H_ */
