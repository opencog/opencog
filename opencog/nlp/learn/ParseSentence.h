/*
 * opencog/nlp/learn/ParseSentence.h
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

#ifndef _PARSE_SENTENCE_H
#define _PARSE_SENTENCE_H

namespace opencog
{
    class AtomSpace;

    class WordPair {
    public:
        float           edge_weight;
        int             left_index;
        Handle          left_node;
        int             right_index;
        Handle          right_node;
        WordPair(double weight, int left, int right) : edge_weight(weight),
                left_index(left), right_index(right)
            {}
    };
    
    typedef std::vector<WordPair> ParseVector;
    typedef std::vector<std::vector<double>> WeightMatrix;

    void parse_words(   AtomSpace*          atomspace,
                        const WordVector&   words,
                        int                 pair_distance_limit,
                        ParseVector&        parse_result);

    void parse_sentence(    AtomSpace*          atomspace,
                            const std::string&  sentence,
                            int                 pair_distance_limit,
                            ParseVector&        parse_results);

    bool dump_pair_weights( AtomSpace*          as,
                            std::string&        file_name,
                            std::string&        sentence,
                            int                 pair_distance_limit,
                            std::string&        error);
}

#endif // _PARSE_SENTENCE_H
