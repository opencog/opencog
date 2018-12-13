/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * Written by Joel Pitt <joel@fruitionnz.com>
 * All Rights Reserved
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

#ifndef _OPENCOG_HDEMO_PATTERN_MATRIX_H
#define _OPENCOG_HDEMO_PATTERN_MATRIX_H

#include <vector>

#include <sys/types.h>

#include <opencog/atoms/truthvalue/AttentionValue.h>
#include <opencog/util/RandGen.h>

namespace opencog
{

extern RandGen* patternRng;


class Pattern : public std::vector< int >
{

private:
    // Height and width of pattern
    int width;
    int height;

    opencog::RandGen *rng;
    std::vector<bool> *mask;

public:

    /**
     * Constructor
     *
     * @param width
     * @param height
     * @param density of random ones
     */
    Pattern(int w, int h, float density = 0.0f);

    Pattern(const Pattern &src) : std::vector<int>(src), 
        width(src.width), height(src.height), rng(patternRng), mask(NULL) {
            if (src.mask) mask = new std::vector<bool>(*(src.mask));
        };

    ~Pattern();

    Pattern & operator = (const Pattern & other) {
        if (this != &other) {
            // protect against invalid self-assignment
            width = other.width;
            height = other.height;
            rng = patternRng;
            mask = NULL;
            if (other.mask) mask = new std::vector<bool>(*(other.mask));
            std::vector<int>::operator=(other);
        }
        return *this;
    }

    /**
     * Similarity based on Hamming distance to another Pattern.
     *
     * @param p Pattern to compare with.
     * @return hamming distance as proportion of total possible (i.e.
     * len(Pattern).
     */
    float hammingSimilarity(const Pattern &p);

    /**
     * Number of bits that mismatch between this and another pattern.
     *
     * @param p Pattern to compare with.
     * @return number of mismatching bits.
     */
    int bitErrors(const Pattern &p);
    
    /**
     * binarise pattern by making values > threshold 1, otherwise 0.
     *
     * @param threshold
     * @return new pattern of 1 and 0.
     */
    Pattern binarisePattern(AttentionValue::sti_t threshold);

    /**
     * Mutate pattern, each value having error chance of mutating to 0, or
     * 0 to 1.
     *
     * @param error rate of mutation
     * @return new mutated pattern
     */
    Pattern mutatePattern(float error);

    /**
     * Mutate a certain number of bits in pattern.
     *
     * @param error number of bit errors
     * @return new mutated pattern
     */
    Pattern mutatePattern(unsigned int error);

    int getWidth();
    int getHeight();
    bool isEmpty();
    void setMask(const std::vector<bool>& _mask);
    bool isMasked(uint i) const;

	int activity();

    /**
     * Generate random patterns.
     *
     * @param amount number of patterns to generate.
     * @param width
     * @param height
     * @param density the probability of value 1.
     * @return vector of generated patterns.
     */
    static std::vector< Pattern > generateRandomPatterns(int amount, int w, int h, float density);

    /**
     * Mutate a series of patterns.
     *
     * @param patterns to mutate.
     * @param mutation rate.
     * @return new mutated patterns.
     */
    static std::vector< Pattern > mutatePatterns( std::vector< Pattern > &patterns, float error);

    /**
     * Load a series of patterns from a file. The format must use O (letter
     * o) as the active nodes, and a space for inactive nodes. The patterns
     * must be square and separated by a blank line.
     *
     * @param file to load
     * @param size equal to width or height (patterns must be square)
     * @return the loaded patterns.
     */
    static std::vector< Pattern > loadPatterns( std::string fn, int size);

    bool operator==(const Pattern& b) const;
};

} // namespace opencog

#endif // _OPENCOG_HDEMO_PATTERN_MATRIX_H
