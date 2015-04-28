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

#include "Pattern.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <stdlib.h>
#ifndef WIN32
#include <sys/time.h>
#endif

#include <opencog/util/platform.h>
#include <opencog/atomspace/AttentionValue.h>
#include <opencog/util/mt19937ar.h>

using namespace std;
using namespace opencog;

RandGen* opencog::patternRng = new MT19937RandGen(1);

Pattern::Pattern(int w, int h, float density)
{
    width = w;
    height = h;
    rng = patternRng;
    mask = NULL;

    for (int i = 0; i < width*height; i++) {
        push_back(rng->randfloat() < density);
    }
}

Pattern::~Pattern()
{
    if (mask) delete mask; 
}


Pattern Pattern::binarisePattern(AttentionValue::sti_t vizThreshold)
{
    Pattern out(width, height);
    Pattern::iterator out_i = out.begin();
    Pattern::iterator i;

    for (i = begin(); i != end(); ++i) {
        int val = *i;
        *out_i = (int) (val >= vizThreshold) ;
        ++out_i;
    }
    return out;
}

void Pattern::setMask(const std::vector<bool>& _mask)
{ mask = new std::vector<bool> (_mask); }

bool Pattern::isMasked(uint i) const
{
    if (mask && (*mask)[i]) return true;
    else return false;
}

bool Pattern::operator==(const Pattern& b) const
{
    if (width != b.width &&
        height != b.height)
        return false;
    for (uint i = 0; i < size(); i++) {
        if (mask && (*mask)[i]) continue;
        if (b.mask && (*b.mask)[i]) continue;
        if (operator[](i) != b[i])
            return false;
    }
    return true;
}

std::vector< Pattern > Pattern::generateRandomPatterns(int amount, int width, int height, float density)
{
    std::vector< Pattern > patterns;

    for (; amount > 0; amount--) {
        Pattern p(width, height, density);
        patterns.push_back(p);
    }
    return patterns;
}


std::vector< Pattern > Pattern::mutatePatterns(std::vector< Pattern > &patterns, float error)
{
    std::vector< Pattern >::iterator i;
    std::vector< Pattern > mutants;

    for (i = patterns.begin(); i != patterns.end(); ++i) {
        Pattern p = (*i).mutatePattern(error);
        mutants.push_back(p);
    }
    return mutants;


}

Pattern Pattern::mutatePattern(float error)
{
    Pattern result(width, height);
    Pattern::iterator r_i = result.begin();
    Pattern::iterator p;
    if (error >= 1) return mutatePattern((unsigned int)error);

    // mutating the pattern so that none of the original
    // nodes are active makes no sense so avoid it.
    int maxToRemove = activity() - 1;

    for (p = begin(); p != end(); ++p) {
        int val = *p;
        // Flip bit with error probability
        if (rng->randfloat() < error) {
            if (val) {
                if (maxToRemove > 0) {
                    val = !val;
                    maxToRemove--;
                }
            } else {
                val = !val;
            }
        }
        *r_i = val; ++r_i;
    }
    return result;
}

Pattern Pattern::mutatePattern(unsigned int error)
{
    Pattern result(*this);
    // mutating the pattern so that none of the original
    // nodes are active makes no sense so avoid it.
    int maxToRemove = activity() - 1;
    if (error > size()) error = size();

    while (error > 0) {
        // Get random index to mutate
        // -1 from size because at least one node should remain
        int index = (int) (rng->randfloat() * (size() - 1));
        int val = (*this)[index];
        // Check if this position has already been mutated
        if (val != result[index]) continue;
        // Flip bit
        if (val) {
            if (maxToRemove > 0) {
                val = !val;
                maxToRemove--;
            }
        } else {
            val = !val;
        }
        result[index] = val;
        error--;
    }
    return result;
}

bool Pattern::isEmpty()
{
    Pattern::iterator p;

    for (uint p = 0; p < size(); p++) {
        if (isMasked(p)) continue;
        if (operator[](p) != 0)
            return false;
    }
    return true;
}

float Pattern::hammingSimilarity(const Pattern &a)
{
    float diff = bitErrors(a);
    return 1.0f - (diff / size());

}

int Pattern::bitErrors(const Pattern &a)
{
    int errors = 0;
    if (a.size() != size())
        return -1;

    for (unsigned int i = 0; i < size(); i++) {
        // If either pattern is masked for this node,
        // then don't contribute the difference to
        // diff
        if (isMasked(i) || a.isMasked(i)) continue;
        // If the nodes are not the same, increment diff
        if ((*this)[i] != a[i]) errors++;
    }

    return errors;
}

std::vector< Pattern > Pattern::loadPatterns( std::string fn, int size)
{
    std::ifstream in;
    char x;
    std::vector< Pattern > patterns;

    cout << "loading file" << endl;

    in.open(fn.c_str());
    if (!in) {
        cout << "cannot open file " << fn << endl;
        exit(1);
    }

    while (in) {

        int row = 0, col = 0;
        Pattern p(size, size);
        while (row < size) {
            while (col < size) {
                if (in) in.get(x);

                if (x == 'O') {
                    p[row*size + col] = 1;
                    col++;
                } else if (x == ' ' || x == '.') {
                    p[row*size + col] = 0;
                    col++;
                } else if (x == '\n') {
                    row++;
                    col = size;
                } else
                    cout << "unknown character in file" << endl;
            }
            col = 0;
        }
        // retreive blank line
        if (in) in.get(x);

        patterns.push_back(p);

    }
    in.close();

    cout << "num patterns = " << patterns.size() << endl;
    return patterns;

}

int Pattern::activity()
{
    Pattern::iterator p;
	int total = 0;

    for (size_t p = 0; p < size(); p++) {
        if (isMasked(p)) continue;
		total += operator[](p);
    }
    return total;

}
