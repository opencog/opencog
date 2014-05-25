/*
 * opencog/learning/moses/moses/ant_scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _ANT_SCORING_H
#define _ANT_SCORING_H

#include <opencog/util/numeric.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

#include "../moses/types.h"
#include "../scoring/scoring.h"

using namespace ant_combo;
using namespace opencog::moses;

#define MIN_FITNESS -1.0e10


static const int ANT_X = 32;
static const int ANT_Y = 32;
static const char init_trail[ANT_Y][ANT_X+1] = {
    " 888                            ",
    "   8                            ",
    "   8                     888    ",
    "   8                    8    8  ",
    "   8                    8    8  ",
    "   8888 88888        88         ",
    "            8                8  ",
    "            8       8           ",
    "            8       8           ",
    "            8       8        8  ",
    "                    8           ",
    "            8                   ",
    "            8                8  ",
    "            8       8           ",
    "            8       8     888   ",
    "                 8     8        ",
    "                                ",
    "            8                   ",
    "            8   8       8       ",
    "            8   8          8    ",
    "            8   8               ",
    "            8   8               ",
    "            8             8     ",
    "            8          8        ",
    "   88  88888    8               ",
    " 8              8               ",
    " 8              8               ",
    " 8      8888888                 ",
    " 8     8                        ",
    "       8                        ",
    "  8888                          ",
    "                                "
};

struct AntFitnessFunction : unary_function<combo_tree, score_t> {

    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    enum Direction { north = 0, east = 1, south = 2, west = 3 };
    static void turn_left(Direction& d);
    static void turn_right(Direction& d);
    static void reverse(Direction& d);

    AntFitnessFunction(int steps = 600);

    score_t operator()(const combo_tree& tr) const;

    bool is_turn_left(builtin_action a) const;
    bool is_turn_right(builtin_action a) const;
    bool is_move_forward(builtin_action a) const;

    int eval(sib_it it, int& x, int& y, Direction& facing, int& at_time,
             char trail[ANT_Y][ANT_X+1]) const;

private:
    const int _steps;
};

struct AntFitnessEstimator : public AntFitnessFunction
{
    AntFitnessEstimator(int steps = 600, int noise = 0);

    score_t operator()(const combo_tree& tr) const;

private:
    const int _noise;
};

struct ant_bscore : public bscore_base
{
    ant_bscore(float simplicity_pressure = 1.0);

    behavioral_score operator()(const combo_tree& tr) const;

    behavioral_score best_possible_bscore() const;

private:
    float _simplicity_pressure;
    AntFitnessFunction _aff;
};

#endif
