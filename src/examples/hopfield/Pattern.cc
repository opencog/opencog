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

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <AttentionValue.h>
#include <mt19937ar.h>

#include "Pattern.h"

using namespace std;

Util::RandGen* patternRng = new Util::MT19937RandGen(1);

Pattern::Pattern(int w, int h, float density)
{
    width = w;
    height = h;
    rng = patternRng;

    for (int i = 0; i < width*height; i++) {
        push_back(rng->randfloat() < density);
    }
}

Pattern::~Pattern()
{
}


Pattern Pattern::binarisePattern(AttentionValue::sti_t vizThreshold)
{
    Pattern out(width, height);
    Pattern::iterator out_i = out.begin();
    Pattern::iterator i;

    for (i = begin(); i != end(); i++) {
        int val = *i;
        *out_i = (int) (val >= vizThreshold) ;
        out_i++;
    }

    return out;
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

    for (i = patterns.begin(); i != patterns.end(); i++) {
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

    for (p = begin(); p != end(); p++) {
        int val = *p;
        // Flip bit with error probability
        if (rng->randfloat() < error) {
            val = !val;
        }
        *r_i = val; r_i++;
    }
    return result;
}

float Pattern::hammingSimilarity(const Pattern &a)
{
    float diff = 0.0f;
    if (a.size() != size())
        return -1.0f;

    for (unsigned int i = 0; i < size(); i++) {
        if ((*this)[i] != a[i]) diff++;
    }

    return 1.0f - (diff / size());

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
                } else if (x == ' ') {
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
