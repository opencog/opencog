/*
 * opencog/nlp/learn/ParseSentence.cc
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
#include <ctime>

#include "unicode/unistr.h"
#include "unicode/locid.h"
#include "unicode/brkiter.h"

#include <opencog/nlp/types/atom_types.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/CountTruthValue.h>

#include "ObserveSentence.h"
#include "ParseSentence.h"

using namespace opencog;

#define NO_EDGE SIZE_MAX
#define NO_WEIGHT (-999.9)

void eat_my_parameters(...) {}

#ifdef DEBUG_MAIN
    #define DEBUG_PRINT printf
#else
    #define DEBUG_PRINT if (false) eat_my_parameters
#endif

namespace opencog {


typedef std::vector<std::vector<double>> WeightMatrix;

#ifdef DEBUG_MAIN
void print_pair_weights(const WordVector&   words,
                        const WeightMatrix& pair_weights)
{
    size_t total_words = words.size();

    // Print the pairs and weights.
    printf("\nPair Weight Matrix\n");
    printf("------------------\n\n");
    printf("%10s", "");
    for (size_t column = 0; column < total_words; column++)
        printf("   %8s", words[column].c_str());
    printf("\n");
    printf("%10s", "");
    for (size_t column = 0; column < total_words; column++)
        DEBUG_PRINT(" %10s", " --------");
    printf("\n");
    
    for (size_t row = 0; row < total_words; row++)
    {
        printf("  %-8s", words[row].c_str());
        for (size_t column = 0; column < total_words; column++)
        {
             double fmi = pair_weights[row][column];
            if (fmi != NO_WEIGHT)
                printf("   %8.1f", fmi);
            else
                printf("         x ");
        }
        printf("\n");
    }
    printf("\n");
}
#endif // DEBUG_MAIN

void load_pair_weights( AtomSpace*          as,
                        const HandleSeq&    word_handles,
                        int                 pair_distance_limit,
                        WeightMatrix&       pair_weights)
{
    size_t total_words = word_handles.size();
    if (total_words <= 0)
        return;

    Handle predicate = as->add_node(PREDICATE_NODE, "*-Sentence Word Pair-*");
    Handle mi_key = as->add_node(PREDICATE_NODE, "*-Mutual Info Key-*");

    double fmi;
    for (size_t left = 0; left < total_words - 1; left++)
    {
        // Compute the last word accounting for our pair distance limits.
        size_t last_word;
        if (pair_distance_limit)
        {
            last_word = left + 1 + pair_distance_limit;
            if (total_words < last_word)
                last_word = total_words;
        }
        else
        {
            last_word = total_words;
        }

        // Get and cache the value for this pair...
        for (size_t right = left + 1; right < last_word; right++)
        {
            // Get the valuation for the mutual information key.
            Handle pair = as->add_link(LIST_LINK, word_handles[left], word_handles[right]);
            Handle evaluation = as->add_link(EVALUATION_LINK, predicate, pair);
            try {
                ProtoAtomPtr proto = evaluation->getValue(mi_key);
                fmi = FloatValueCast(proto)->value()[1];
            }
            catch (...)
            {
                fmi = NO_WEIGHT;
                std::cerr << "Missing *-Sentence Word Pair-* for <"  << 
                        word_handles[left]->getName().c_str() << "," <<
                        word_handles[right]->getName().c_str() << ">" << std::endl;
            }
            pair_weights[left][right] = fmi;
        }    
    }
}

static int parse_recursion = 0;

void add_parse( ParseVector&    parse_result,
                double          weight,
                int             left,
                int             right)
{
#ifdef DEEP_DEBUG
    DEBUG_PRINT("Adding parse pair [%d][%d]\n", left, right);
#endif
    parse_result.push_back(WordPair(weight, left, right));
}

double connect_range(   std::string&            indent,
                        WeightMatrix&           pair_weights,
                        const WordVector&       words,
                        size_t                  start,
                        size_t                  end,
                        bool                    save_parse,
                        ParseVector&            parse_result)
{
    double connected_weight;
    size_t left = start - 1;
    size_t right = end + 1;

    // Bump the indent.
    parse_recursion++;
    indent += " >";

    // If this is a single...
    if ( start == end )
    {
        DEBUG_PRINT("%s connect single [%d] %-10s\n",
                indent.c_str(),  (int) start, words[start].c_str());
    }
    else
    {
        DEBUG_PRINT("%s connect range [%d] %-10s to [%d] %-10s\n",
                indent.c_str(),  (int) start, words[start].c_str(),
                (int) end, words[end].c_str());
    }

    // Get the left connected weight.
    double left_weight = pair_weights[left][start];
    DEBUG_PRINT("%s   left  [%d] %-10s [%d] %-10s = %4.02f\n",
            indent.c_str(),  (int) left, words[left].c_str(), 
            (int) start, words[start].c_str(), left_weight);

    // And right connected weight.
    double right_weight = pair_weights[end][right];
    DEBUG_PRINT("%s   right [%d] %-10s [%d] %-10s = %4.02f\n",
            indent.c_str(),  (int) end, words[end].c_str(), 
            (int) right, words[right].c_str(), right_weight);

    // Return the larger weight.
    if (left_weight > right_weight)
    {
        DEBUG_PRINT("%s   connect left\n", indent.c_str());
        connected_weight = left_weight;

        // Remember the parse.
        if (save_parse)
            add_parse(parse_result, connected_weight, left, start);
    }
    else
    {
        DEBUG_PRINT("%s   connect right\n", indent.c_str());
        connected_weight = right_weight;
        
        // Remember the parse.
        if (save_parse)
            add_parse(parse_result, connected_weight, end, right);
    }

    parse_recursion--;
    indent.resize(indent.size() - 2);

    return connected_weight;
}

void find_max_pair_index(   std::string&        indent,
                            WeightMatrix&       pair_weights,
                            const WordVector&   words,
                            int                 pair_distance_limit,
                            size_t              left,
                            size_t              end,
                            size_t&             max_pair_index,
                            double&             max_pair_weight)
{
    // Find the maximum pair weight for the left word.
    max_pair_index = 0;
    max_pair_weight = NO_WEIGHT;

    // Constrain to the pair distance limit.
    size_t last_right = left + pair_distance_limit;
    if (end < last_right)
        last_right = end;
    for (size_t right = left + 1; right <= end; right++)
    {
        double pair_weight = pair_weights[left][right];
        if (pair_weight > max_pair_weight)
        {
            max_pair_index = right;
            max_pair_weight = pair_weight;
        }
    }

#ifdef DEEP_DEBUG
    DEBUG_PRINT("%s max for [%d] %-10s to [%d] is [%d] = %4.02f\n",
            indent.c_str(),  (int) left, words[left].c_str(),
            (int) end, (int) max_pair_index, max_pair_weight);
#endif
}

size_t potential_cross( std::string&                indent,
                        const std::vector<size_t>&  max_pair_indices,
                        const WordVector&           words,
                        size_t                      start,
                        size_t                      current_max_right)
{
    // Check for potential crosses from right to left since
    // if we find a cross and the cross has a greater weighted
    // tree we will be lowering the maximum in the pair_weights
    // matrix so that future parses do not have to do this check
    // again. Since there may be more than one potential cross
    // we will miss some potential high weight pairs if we move
    // the current search too far left in one go.
    // 
    // By searching from right to left, we will find the
    // rightmost potential cross first. We will eliminate it and
    // search again for a cross below.
    for (int left = current_max_right - 1; left >= (int) start; left--)
    {
        size_t left_max = max_pair_indices[left - start];

#ifdef DEEP_DEBUG
        DEBUG_PRINT("%s checking cross for left [%d] %-10s against [%d] with [%d]\n",
                indent.c_str(), left, words[left].c_str(),
                (int) current_max_right, (int) left_max);
#endif
        // If this pair is to the right of the check then
        // we have a potential crossed pair.
        if (left_max > current_max_right)
        {
#ifdef DEEP_DEBUG
            DEBUG_PRINT("%s found cross, left [%d] %-10s crosses [%d] with [%d]\n",
                    indent.c_str(), left, words[left].c_str(),
                    (int) current_max_right, (int) left_max);
#endif
            return (size_t) left;
        }
    }

    return (size_t) false;
}

#define DONT_CHECK_CROSSES false
#define SAVE_PARSE true
#define DONT_SAVE_PARSE false

double parse_fragment(  std::string&            indent,
                        WeightMatrix&           pair_weights,
                        const WordVector&       words,
                        int                     pair_distance_limit,
                        size_t                  start,
                        size_t                  end,
                        bool                    save_parse,
                        ParseVector&            parse_result,
                        bool                    check_crosses = true)
{
    size_t fragment_size = end - start;
    std::vector<size_t> max_pair_indices(fragment_size);
    std::vector<double> max_pair_weights(fragment_size);
    size_t max_pair_index;
    double max_pair_weight;

    parse_recursion++;
    if (parse_recursion > (int) words.size())
    {
        printf("ERROR: Parse recursion exceeds sentence length...\n");
        exit(1);
    }

    // Bump the indent.
    size_t indent_size_on_entry = indent.size();
    indent += " >";

    DEBUG_PRINT("%s parsing fragment [%d] %-10s to [%d] %-10s\n", indent.c_str(), 
            (int) start, words[start].c_str(),  (int) end, words[end].c_str());

    // Determine the maximum weight pairs. NOTE: This is where the
    // pair distance is checked. That keeps the code in the sections below 
    // simpler as it doesn't have to account for pair distances limits if
    // the max_pair_index already accounts for this limit. 
    size_t left_cache_index;
    for (size_t left = start; left < end; left++)
    {
        // Save the maximum pair index and weight.
        find_max_pair_index(indent, pair_weights, words, pair_distance_limit,
                    left, end, max_pair_index, max_pair_weight);
        left_cache_index = left - start;
        max_pair_indices[left_cache_index] = max_pair_index;
        max_pair_weights[left_cache_index] = max_pair_weight;
#ifdef DEEP_DEBUG
        printf("%s   max for [%d] %-10s [%d] %-10s = %4.02f\n",
                indent.c_str(),  (int) left, words[left].c_str(), 
                (int) max_pair_index, words[max_pair_index].c_str(), max_pair_weight);
#endif
    }

    // Now that we've computed the maximum weight pairs for each
    // word in the fragment, we will proceed from left to right
    // determining how to connect these parts.
    //
    // Here we check for crossing maximums, i.e. any pairs that would
    // cross this maximum. There are three cases here:
    //
    // 1) The maximum edge for left is at index: left + 1. A cross
    //    cannot exist, so we'll skip checking the maximum for
    //    left + 1 and start at the next index instead.
    // 
    // 2) All the maximums for potential crossing pairs are
    //    for pairs with an index lower than the maximum weighted
    //    word for the left pair.
    //
    // 3) At least one word with an index between the left word
    //    and the left word's maximum has it's maximum at a pair
    //    that crosses the left word's maximum, i.e. the index
    //    for this crossing pair is greater than the index for the 
    //    left word's maximum. 
    //
    // For case 1), no edge crossing is possible, for case 2), no
    // cross is indicated, so in both these cases we can add the
    // maximum to the parse results. In case two, we also update
    // the right boundary, and recompute the maximum and index for
    // this word.
    //
    // In case 3), we must determine which of the conflicting edges 
    // results in a greater total spanning tree weight. 
    //
    // Consider the sentence, "Note that this crosses the left's
    // maximum to end." with the following maximum edges:
    //
    // Max MI Edge Table
    // -----------------
    //          Note that this crosses the left's maximum to end
    // Note                                          5
    // that                 2  
    // this                                                   3
    // crosses                          1
    // the                                    1
    // left's                                        2
    // maximum                                                2
    // to                                                 2
    // end
    //
    // This represents a directed graph from the first column to
    // the corresponding edge column. As can be seen, there is
    // a conflict between the edge from "Note" to "maximum" and
    // "this" to "end" because these maxima cross and this cross
    // must be eliminated.
    // 
    //                  /-----------------------------\       '
    //        /-------------------------------         \      '
    //       /        /                       \         \     '
    //    Note that this crosses the left's maximum to end.
    //    1     2    3      4     5    6       7     8  9
    // 
    // The word "this" at position 3, has a maximum edge that will 
    // cross to word "end" at position 9, since "Note" at position 1
    // has a maximum edge at position 7.
    // 
    double fragment_weight = 0;
    size_t left = start;
    while (left < end)
    {
        indent.resize(indent_size_on_entry);
        DEBUG_PRINT("%s parsing left [%d] %-10s\n",
                indent.c_str(),  (int) left, words[left].c_str());
        indent += "  ";

        // Get the maximum pair and weight from our cached list.
        left_cache_index = left - start;
        max_pair_index = max_pair_indices[left_cache_index];
        max_pair_weight = max_pair_weights[left_cache_index];

        // If the left has a maximum that is not the next pair...
        size_t subfragment_start = left + 1;
        int cross_count = false;
        if (max_pair_index > subfragment_start)
        {
            bool saving_parse_on_entry = save_parse;
            save_parse = false;
            
            // We must check and eliminate potential crosses first...
            size_t cross;
            while ( check_crosses &&
                    (cross = potential_cross(indent, max_pair_indices, words, left, max_pair_index)))
            {
                size_t cross_cache_index = cross - start;
                size_t cross_right_index = max_pair_indices[cross_cache_index];
                size_t cross_weight = max_pair_weights[cross_cache_index];
                DEBUG_PRINT("%s edge [%d]__[%d] crosses edge [%d]__[%d]\n",
                        indent.c_str(),  (int) left, (int) max_pair_index,
                        (int) cross, (int) cross_right_index);
                indent += " |";

                // If we get here, we have at least one cross.
                cross_count++;

                // First some terms:
                // 
                // left_left_parse_weight - the weight of the parse from left to it's max_pair_index
                // left_right_parse_weight - the weight of the parse from the left's max_pair_index to 
                //                           the cross's max_pair index
                //
                // right_left_parse_weight - the weight of the parse to the cross
                // right_right_parse_weight - the weight of the parse from the cross to it's max_pair_index
                //
                
                // Determine the parse weight if the left wins...
                double left_left_parse_weight = parse_fragment(indent, pair_weights, words,
                        pair_distance_limit, left, max_pair_index, save_parse,
                        parse_result, DONT_CHECK_CROSSES);
                double left_right_parse_weight = parse_fragment(indent, pair_weights, words,
                        pair_distance_limit, max_pair_index + 1, cross_right_index,
                        save_parse, parse_result, DONT_CHECK_CROSSES);
                double left_parse_weight = left_left_parse_weight + 
                        left_right_parse_weight + max_pair_weight;

                // Determine the parse weight if the right wins...
                double right_left_parse_weight = parse_fragment(indent, pair_weights, words,
                        pair_distance_limit, left, cross, save_parse,
                        parse_result, DONT_CHECK_CROSSES);
                double right_right_parse_weight = parse_fragment(indent, pair_weights, words,
                        pair_distance_limit, cross + 1, cross_right_index, save_parse,
                        parse_result, DONT_CHECK_CROSSES);
                double right_parse_weight = right_left_parse_weight +
                        right_right_parse_weight + cross_weight;

                // The parse with the greatest total fragment weight wins.

                // If the left cross wins...
                if (left_parse_weight > right_parse_weight)
                {
                    DEBUG_PRINT("%s removing crossing edge [%d]__[%d]\n",
                            indent.c_str(),  (int) cross, (int) cross_right_index);

                    // Remove the cross by constricting the range of the maximum pair for the
                    // right so it cannot cross the winning left pair.
                    size_t new_cross_limit = max_pair_index;
                    find_max_pair_index(indent, pair_weights, words, pair_distance_limit,
                            cross, max_pair_index, max_pair_index, max_pair_weight);
                    max_pair_indices[cross_cache_index] = max_pair_index;
                    max_pair_weights[cross_cache_index] = max_pair_weight;

                    DEBUG_PRINT("%s found new max pair for [%d] %-10s up to [%d] at [%d] = %4.02f\n",
                            indent.c_str(),  (int) cross, words[left].c_str(),
                            (int) new_cross_limit, (int) max_pair_index, max_pair_weight);
                }
                // otherwise, the right cross wins...
                else
                {
                    DEBUG_PRINT("%s removing crossing edge [%d]__[%d]\n",
                            indent.c_str(),  (int) left, (int) max_pair_index);

                    // Remove the cross by constricting the range of the maximum pair for the
                    // left so it cannot cross the winning cross pair.
                    find_max_pair_index(indent, pair_weights, words, pair_distance_limit, 
                            left, cross, max_pair_index, max_pair_weight);
                    max_pair_indices[left_cache_index] = max_pair_index;
                    max_pair_weights[left_cache_index] = max_pair_weight;

                    DEBUG_PRINT("%s found new max pair for [%d] %-10s up to [%d] at [%d] = %4.02f\n",
                            indent.c_str(),  (int) left, words[left].c_str(),
                            (int) cross, (int) max_pair_index, max_pair_weight);
                }

                indent.resize(indent.size() - 2);

            } // while (checking potential crosses)

            // Restore the saving parse flag...
            save_parse = saving_parse_on_entry;
        }

        if (check_crosses && cross_count)
        {
            DEBUG_PRINT("%s %d crosses eliminated for left [%d].\n", indent.c_str(),
                    cross_count, (int) left);
        }

        // At this point, any potential crosses will have been eliminated by adjusting the
        // max_pair_indexes to remove the conflicting crosses so that only the edge that
        // builds the maximum subfragment parse will remain.

        // So now we handle subparsing and connection of subparsing without worrying about
        // crosses.
        if (max_pair_index > subfragment_start)
        {
            // Remember the parse.
            if (save_parse)
                add_parse( parse_result, fragment_weight, left, max_pair_index);

            // Parse the contained fragment and connect it one
            // to the left or right.
            size_t subfragment_end = max_pair_index - 1;
            if (subfragment_end > subfragment_start)
            {
                DEBUG_PRINT("%s parsing subfragment of [%d] %-10s [%d] %-10s = %4.02f\n",
                        indent.c_str(),  (int) left, words[left].c_str(), 
                        (int) max_pair_index, words[max_pair_index].c_str(), max_pair_weight);

                // Now parse and connect the subfragment.
                size_t range_start = left + 1;
                size_t range_end = max_pair_index - 1;
                double subfragment_weight = parse_fragment(indent, pair_weights, words,
                        pair_distance_limit, range_start, range_end, save_parse, parse_result);
                double connection_weight = connect_range(indent, pair_weights, words, 
                        range_start, range_end, save_parse, parse_result);
                fragment_weight += subfragment_weight + connection_weight;
            }
            else
            {
                DEBUG_PRINT("%s connecting single of [%d] %-10s [%d] %-10s = %4.02f\n",
                        indent.c_str(),  (int) left, words[left].c_str(), 
                        (int) max_pair_index, words[max_pair_index].c_str(), max_pair_weight);
                fragment_weight += connect_range(indent, pair_weights, words,
                        subfragment_start, subfragment_start, save_parse, parse_result);
            }

            // We already parsed the left as a contained parse. So we can
            // skip ahead to the next position.
            left = max_pair_index;
        }
        else
        {
            DEBUG_PRINT("%s max at +1   [%d] %-10s [%d] %-10s = %4.02f\n",
                    indent.c_str(),  (int) left, words[left].c_str(), 
                    (int) max_pair_index, words[max_pair_index].c_str(), max_pair_weight);
            fragment_weight += max_pair_weight;

            // Remember the parse.
            if (save_parse)
                add_parse(parse_result, fragment_weight, left, max_pair_index);

            // Check the next left.
            left++;
        }
    }

    parse_recursion--;
    indent.resize(indent_size_on_entry);
    return fragment_weight;
}

/*

parse_words - builds a parse vector for the words. A parse vector is 
an ordered list of word pairs spanning the highest non-crossing
mutual information tree. 

*/
void parse_words(   AtomSpace*          atomspace,
                    const WordVector&   words,
                    int                 pair_distance_limit,
                    ParseVector&        parse_result)
{
    if (words.size() <= 0)
        return;

    // Cache the word nodes.
    HandleSeq word_handles;
    size_t total_words = words.size();
    for (size_t index = 0; index < total_words; index++)
        word_handles.push_back(atomspace->add_node(WORD_NODE, words[index]));

    // Cache the pair weights since we will need to examine each pair as part
    // of the parsing anyway,  this will allow us to not load compare pairs more
    // than once since we can just use the computed weight cache.
    WeightMatrix pair_weights(total_words, std::vector<double>(total_words, NO_WEIGHT));
    load_pair_weights(atomspace, word_handles, pair_distance_limit, pair_weights);

#ifdef DEBUG_MAIN
    print_pair_weights(words, pair_weights);
#endif

    // Now that we have setup the word handles and pair weights cache we can
    // do our recursive fragment parsing, passing in indexes for the whole sentence.
    std::string indent = "";
    double parse_weight = parse_fragment(indent, pair_weights, words, pair_distance_limit,
            0, total_words - 1, SAVE_PARSE, parse_result);
    DEBUG_PRINT("Parse total weight = %.2f.\n", parse_weight);
}

void parse_sentence(    AtomSpace*      atomspace,
                        std::string&    sentence,
                        int             pair_distance_limit,
                        ParseVector&    parse_results)
{
    WordVector words;

    // Break the sentence up into words.
    break_sentence_into_words(sentence, words);

    // Parse the words.
    parse_words(atomspace, words, pair_distance_limit, parse_results);
}


} // namespace opencog


#ifdef TEST_MAIN
void test_parse_sentence(   AtomSpace*      atomspace,
                            std::string&    sentence,
                            int             pair_distance_limit = NO_PAIR_DISTANCE_LIMIT)
{
    ParseVector     parse_results;
    WordVector      words;

    // Break the sentence up into words.
    break_sentence_into_words(sentence, words);

    // Parse the words.
    parse_words(atomspace, words, pair_distance_limit, parse_results);

    // Print out the parse results.
    for (auto & pair : parse_results)
    {
        printf(" MI = %0.2f, [%d] %10s -  [%d] %10s\n", pair.edge_weight, 
                pair.left_index, words[pair.left_index].c_str(),
                pair.right_index, words[pair.right_index].c_str());
    }
    printf("\n");

}

void add_test_word_atoms(AtomSpace*    as,
                         std::string&  sentence)
{
    WordVector words;
    ParseVector parse;

    // Break the sentence up into words.
    break_sentence_into_words(sentence, words);

    // Create all the word nodes.
    HandleSeq word_handles;
    size_t total_words = words.size();
    for (size_t index = 0; index < total_words; index++)
        word_handles.push_back(as->add_node(WORD_NODE, words[index]));

    // Create the pair atoms so our timings don't include atom creation times as
    // they will be loaded for actual parsing.
    WeightMatrix pair_weights(total_words, std::vector<double>(total_words, NO_WEIGHT));
    load_pair_weights(as, word_handles, NO_PAIR_DISTANCE_LIMIT, pair_weights);
}

void add_test_word_weights( AtomSpace*      as,
                            std::string&    sentence,
                            double          weights[8][8])
{
    WordVector words;
    ParseVector parse;
    // Create the predicate and schema nodes only once.
    Handle predicate = as->add_node(PREDICATE_NODE, "*-Sentence Word Pair-*");
    Handle mi_key = as->add_node(PREDICATE_NODE, "*-Mutual Info Key-*");

    // Break the sentence up into words.
    break_sentence_into_words(sentence, words);
    if (words.size() <= 0)
        return;

    // Loop over all the words adding the pair weights.
    int total_words = words.size();
    for (int left = 0; left < total_words - 1; left++)
        for (int right = left + 1; right < total_words; right++)
        {
            // Create the pair atoms.
            Handle left_word = as->add_node(WORD_NODE, words[left]);
            Handle right_word = as->add_node(WORD_NODE, words[right]);
            Handle pair = as->add_link(LIST_LINK, left_word, right_word);
            Handle evaluation = as->add_link(EVALUATION_LINK, predicate, pair);

            // Set the mutual information weight.
            double weight = weights[left][right];
            std::vector<double> float_list = { (double) 0.0, weight };
            ProtoAtomPtr proto = createFloatValue( float_list );
            evaluation->setValue(mi_key, proto);
        }
}

void print_atomspace(AtomSpace* as)
{
    HandleSeq handles;
    as->get_handles_by_type(back_inserter(handles), ATOM, true);
    for (const Handle& h : handles) {
        std::cout << h->toString() << std::endl;
    }
 
}

int main(int argc, char *argv[])
{
    AtomSpace       atomspace;
    AtomSpace*      as = &atomspace;
    std::string     test_sentence("The dog barked.");
    std::clock_t    start;
    double          elapsed;

    double          test_0_weights[8][8] = {
                    // The   dog barked
                       {0.0, 0.5, 0.1} , // The
                       {0.0, 0.0, 1.2} , // dog
                       {0.0, 0.0, 0.0} , // barked
                    };

    printf("\nUpdating weight data.\n");
    add_test_word_weights(as, test_sentence, test_0_weights);

    printf("Parsing sentence '%s'.\n", test_sentence.c_str());
    start = std::clock();
    test_parse_sentence(as, test_sentence);
    elapsed = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Sentence parsed in %0.6f seconds\n", elapsed);

    test_sentence = "The big dog did eat the little dog";
    double          test_1_weights[8][8] = {
                    // The   big  dog  did  eat  the  little  dog
                       {0.0, 0.3, 0.5, 0.2, 0.1, 1.0,    0.5, 0.5} , // The
                       {0.0, 0.0, 1.2, 0.1, 0.1, 0.1,    0.2, 1.2} , // big
                       {0.0, 0.0, 0.0, 0.5, 1.0, 0.5,    0.3, 0.2} , // dog
                       {0.0, 0.0, 0.0, 0.0, 1.4, 0.5,    0.3, 0.2} , // did
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.5,    0.6, 0.7} , // eat
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    1.1, 1.7} , // the
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 1.0} , // little
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 0.0} , // dog
                    };

    printf("\nUpdating weight data.\n");
    add_test_word_weights(as, test_sentence, test_1_weights);

    printf("Parsing sentence '%s'.\n", test_sentence.c_str());
    start = std::clock();
    test_parse_sentence(as, test_sentence);
    elapsed = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Sentence parsed in %0.6f seconds\n", elapsed);

    printf("Parsing sentence '%s'.\n", test_sentence.c_str());
    printf("Pair distance 2.\n");
    start = std::clock();
    test_parse_sentence(as, test_sentence, 2);
    elapsed = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Sentence parsed in %0.6f seconds\n", elapsed);

    double          test_2_weights[8][8] = {
                    // The   big  dog  did  eat  the  little  dog
                       {0.0, 1.0, 0.5, 0.2, 0.1, 0.1,    1.0, 0.5} , // The
                       {0.0, 0.0, 1.2, 0.1, 0.1, 0.1,    0.2, 1.2} , // big
                       {0.0, 0.0, 0.0, 0.5, 1.0, 0.5,    0.3, 0.2} , // dog
                       {0.0, 0.0, 0.0, 0.0, 1.4, 0.5,    0.3, 0.2} , // did
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.5,    0.6, 0.7} , // eat
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    1.0, 1.7} , // the
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 1.0} , // little
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 0.0} , // dog
                    };

    printf("\nUpdating weight data.\n");
    add_test_word_atoms(as, test_sentence);
    add_test_word_weights(as, test_sentence, test_2_weights);

    printf("Parsing sentence '%s'.\n", test_sentence.c_str());
    start = std::clock();
    test_parse_sentence(as, test_sentence);
    elapsed = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Sentence parsed in %0.6f seconds\n", elapsed);

    double          test_3_weights[8][8] = {
                    // The   big  dog  did  eat  the  little  dog
                       {0.0, 0.3, 0.5, 0.2, 0.1, 0.1,    1.5, 0.5} , // The
                       {0.0, 0.0, 1.2, 0.1, 0.1, 0.1,    0.2, 1.2} , // big
                       {0.0, 0.0, 0.0, 0.5, 1.0, 0.5,    0.3, 0.2} , // dog
                       {0.0, 0.0, 0.0, 0.0, 1.4, 0.5,    0.3, 0.2} , // did
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.5,    0.6, 0.7} , // eat
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    1.1, 1.7} , // the
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 1.0} , // little
                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 0.0} , // dog
                    };

    printf("\nUpdating weight data.\n");
    add_test_word_weights(as, test_sentence, test_3_weights);

    printf("Parsing sentence '%s'.\n", test_sentence.c_str());
    start = std::clock();
    test_parse_sentence(as, test_sentence);
    elapsed = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Sentence parsed in %0.6f seconds\n", elapsed);


    exit(0);
}
#endif

