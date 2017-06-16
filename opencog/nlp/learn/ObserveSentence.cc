/*
 * opencog/nlp/learn/ObserveSentence.cc
 *
 * Copyright (C) 2017 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Curtis Faith
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <iostream>
#include <string>
#include <list>

#include "unicode/unistr.h"
#include "unicode/locid.h"
#include "unicode/brkiter.h"

#include <opencog/nlp/types/atom_types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/CountTruthValue.h>

#include "ObserveSentence.h"

using namespace opencog;


void eat_me(...) {}
// #define DEBUG_PRINT printf
#define DEBUG_PRINT if (false) eat_me

namespace opencog {

void break_sentence_into_words( const std::string& sentence,
                                std::vector<std::string>& words)
{
    const icu::UnicodeString unicode_sentence(sentence.c_str(), sentence.length());
    icu::UnicodeString unicode_word;
    std::string word;
    UErrorCode status = U_ZERO_ERROR;
    icu::BreakIterator* bi;

    bi = icu::BreakIterator::createWordInstance(icu::Locale::getUS(), status);
    bi->setText(unicode_sentence);
    int32_t current_break = bi->first();
    
    // Loop over the sentence extracting words.
    int count = 0;
    int32_t last_break = 0;
    while (current_break != icu::BreakIterator::DONE)
    {
 
        int breakType = bi->getRuleStatus();
        if (breakType != UBRK_WORD_NONE)
        {
            // Exclude spaces, punctuation, and the like.

            // Extract the word as a std::string.
            unicode_sentence.extract(last_break, current_break - last_break, unicode_word);
            word.clear();
            unicode_word.toUTF8String(word);

            // Add the word to our list.
            words.push_back(word);
            
            //   A status value 
            DEBUG_PRINT("word break: %d\n", current_break);
            DEBUG_PRINT(".         : %s\n", word.c_str());
            ++count;
        }
        else
        {
            // UBRK_WORD_NONE indicates that the boundary does not start
            // a word or number.
            DEBUG_PRINT("   non-word break: %d\n", current_break);
        }

        last_break = current_break;
        current_break = bi->next();
    }
    delete bi;
}

void increment_count(AtomSpace* as, Handle atom, int increment = 1)
{
    static std::mutex count_mutex;
    std::lock_guard<std::mutex> lock(count_mutex);

    int new_count;

    // Get the new count
    TruthValuePtr tv = atom->getTruthValue();
    if (COUNT_TRUTH_VALUE == tv->getType())
        new_count = tv->getCount() + increment;
    else
        new_count = increment;

    // Create the new truth value with this count.
    tv = CountTruthValue::createTV( tv->getMean(), tv->getConfidence(),
        new_count);
    atom->setTruthValue(tv);
}


/*

create_atoms_for_words - creates atoms for the sentence corresponding to
observations which can be processed by later steps in the language learning 
pipeline. For example, as called by the CogServer's ObserveRequest "observe",
the following command sequence:

    observe -pair_distance 2 "Stop your laughing."

will create structures of the form:

    EvaluationLink
        PredicateNode "*-Sentence Word Pair-*"
        ListLink
            WordNode "Stop"
            WordNode "your"

    EvaluationLink
        PredicateNode "*-Sentence Word Pair-*"
        ListLink
            WordNode "Stop"
            WordNode "laughing"

    EvaluationLink
        PredicateNode "*-Sentence Word Pair-*"
        ListLink
            WordNode "your"
            WordNode "laughing"

    ExecutionLink
        SchemaNode "*-Pair Distance-*"
        ListLink
            WordNode "Stop"
            WordNode "your"
        NumberNode 1

    ExecutionLink
        SchemaNode "*-Pair Distance-*"
        ListLink
            WordNode "Stop"
            WordNode "laughing"
        NumberNode 2

    ExecutionLink
        SchemaNode "*-Pair Distance-*"
        ListLink
            WordNode "your"
            WordNode "laughing"
        NumberNode 1

*/
void create_atoms_for_words(AtomSpace*                  as,
                            std::vector<std::string>&   word_strings,
                            size_t                      pair_distance_limit)
{
    size_t total_words = word_strings.size();
    if (total_words < 1)
        return;

    // Create the predicate and schema nodes only once.
    Handle predicate = as->add_node(PREDICATE_NODE, "*-Sentence Word Pair-*");
    Handle schema = as->add_node(SCHEMA_NODE, "*-Pair Distance-*");

    DEBUG_PRINT("adding atoms for %d words\n", (int) word_strings.size());

    // Create atoms for each word.
    std::vector<Handle> words;
    words.reserve(total_words);
    for (size_t index = 0; index < total_words; index++)
        words.emplace_back(as->add_node(WORD_NODE, word_strings[index]));

    // Create atoms for our pair distances.
    size_t max_pair_distance = total_words - 1;
    if (pair_distance_limit && pair_distance_limit < total_words)
        max_pair_distance = pair_distance_limit;
    std::vector<Handle> distances;
    distances.reserve(max_pair_distance);
    for (size_t index = 1; index <= max_pair_distance; index++)
        distances.emplace_back(as->add_node(NUMBER_NODE, std::to_string(index)));

    // Loop over all the word pairs...
    for (size_t first = 0; first < total_words; first++)
    {
        // Increment the count for the word.
        increment_count(as, words[first]);
        
        // Determine the last pair word for this first word.
        size_t last_pair_word = total_words;
        if (pair_distance_limit &&
            (first + pair_distance_limit + 1) < total_words)
            last_pair_word = first + pair_distance_limit + 1;

        // Create the pair count atoms...
        for (size_t second = first + 1; second < last_pair_word; second++)
        {
            DEBUG_PRINT("pair <%s,%s>\n", word_strings[first].c_str(), word_strings[second].c_str());

            // Record the <first, second> word pair.
            Handle pair = as->add_link(LIST_LINK, words[first], words[second]);
            Handle evaluation = as->add_link(EVALUATION_LINK, predicate, pair);
            increment_count(as, evaluation);

            // Record the distance.
            size_t distance_index = second - first - 1;
            Handle execution = as->add_link(EXECUTION_LINK, schema, pair, 
                    distances[distance_index]);
            increment_count(as, execution);
        }
    }
}

void observe_sentence(  AtomSpace*      atomspace,
                        std::string&    sentence,
                        size_t          pair_distance_limit)
{
    std::vector<std::string> words;

    // Create the corresponding atoms for the words in this sentence.
    break_sentence_into_words(sentence, words);
    create_atoms_for_words(atomspace, words, pair_distance_limit);
}

} // namespace opencog
