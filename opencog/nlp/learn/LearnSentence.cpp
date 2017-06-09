
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

#include "LearnSentence.h"

using namespace opencog;


void eat_me(...) {}
// #define DEBUG_PRINT printf
#define DEBUG_PRINT if (false) eat_me

namespace opencog {

void break_sentence_into_words( const std::string& sentence,
                                std::vector<std::string>& words) {
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
    while (current_break != icu::BreakIterator::DONE) {
 
       	int breakType = bi->getRuleStatus();
        if (breakType != UBRK_WORD_NONE) {
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

        } else {

        	// UBRK_WORD_NONE indicates that the boundary does not start
        	// a word or number.
        	DEBUG_PRINT("   non-word break: %d\n", current_break);
        }

        last_break = current_break;
        current_break = bi->next();
    }
    delete bi;
}

/*
; The structures that get created and incremented are of the form
;
;     EvaluationLink
;         PredicateNode "*-Sentence Word Pair-*"
;         ListLink
;             WordNode "lefty"  -- or whatever words these are.
;             WordNode "righty"
;
;     ExecutionLink
;         SchemaNode "*-Pair Distance-*"
;         ListLink
;             WordNode "lefty"
;             WordNode "righty"
;         NumberNode 3
;
; Here, the NumberNode encdes the distance between the words. It is always
; at least one -- i.e. it is the diffference between their ordinals.
;
(define (update-pair-counts-once PARSE)

    ; Get the scheme-number of the word-sequence number
    (define (get-no seq-lnk)
        (string->number (cog-name (gdr seq-lnk))))

    ; Create and count a word-pair, and the distance.
    (define (count-one-pair left-seq right-seq)
        (define pare (ListLink (gar left-seq) (gar right-seq)))
        (define dist (- (get-no right-seq) (get-no left-seq)))

        (count-one-atom (EvaluationLink pair-pred pare))
        (count-one-atom (ExecutionLink pair-dist pare (NumberNode dist))))
*/

void increment_count(AtomSpace* as, Handle atom)
{
    int new_count;

    // Get the new count
    TruthValuePtr tv = atom->getTruthValue();
    if (COUNT_TRUTH_VALUE == tv->getType())
        new_count = tv->getCount() + 1;
    else
        new_count = 1;

    // Create the new truth value with this count.
    tv = CountTruthValue::createTV( tv->getMean(), tv->getConfidence(),
        new_count);
    atom->setTruthValue(tv);
}


void create_atoms_for_words(AtomSpace* as, std::vector<std::string>& words)
{
    // Create the predicate and schema nodes only once.
    Handle predicate = as->add_node(PREDICATE_NODE, "*-Sentence Word Pair-*");
    Handle schema = as->add_node(SCHEMA_NODE, "*-Pair Distance-*");

    DEBUG_PRINT("adding atoms for %d words\n", (int) words.size());

    // Loop over all the word pairs.
    size_t total_words = words.size();
    for (size_t first = 0; first < total_words; first++)
    {
        // Get the node for the first word and count it.
        Handle first_word = as->add_node(WORD_NODE, words[first]);
        increment_count(as, first_word);
        
        for (size_t second = first + 1; second < total_words; second++)
        {
        DEBUG_PRINT("pair <%s,%s>\n", words[first].c_str(), words[second].c_str());

        // Record the <first,second> word pair.
        Handle second_word = as->add_node(WORD_NODE, words[second]);
        Handle pair = as->add_link(LIST_LINK, first_word, second_word);
        Handle evaluation = as->add_link(EVALUATION_LINK, predicate, pair);

        // Record the distance.
        std::string distance = std::to_string(second - first);
        Handle number = as->add_node(NUMBER_NODE, distance);
        Handle execution = as->add_link(EXECUTION_LINK, schema, pair, number);

        // Increment the counts.
        increment_count(as, evaluation);
        increment_count(as, execution);
        }
    }
}

void learn_sentence(AtomSpace* atomspace, std::string& sentence)
{
    std::vector<std::string> words;

    // Create the corresponding atoms for the words in this sentence.
    break_sentence_into_words(sentence, words);
    create_atoms_for_words(atomspace, words);
}

} // namespace opencog