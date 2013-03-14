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
static char trail[ANT_Y][ANT_X+1];
static const char trail2[ANT_Y][ANT_X+1] = {
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
    static void turn_left(Direction& d) {
        d = (Direction)(((int)d + 3) % 4);
    }
    static void turn_right(Direction& d) {
        d = (Direction)(((int)d + 1) % 4);
    }
    static void reverse(Direction& d) {
        d = (Direction)(((int)d + 2) % 4);
    }

    AntFitnessFunction(int steps = 600) :
            _steps(steps) { }

    result_type operator()(argument_type tr) const {
        //cout << "zz" << endl;
        copy(trail2[0],trail2[0] + sizeof(trail2), trail[0]); //nasty..
        //cout << "top, " << t << endl;
        if (tr.empty())
            return MIN_FITNESS;
        int x = 0, y = 0;
        Direction facing = east;
        int at_time = 0;
        int sc = 0;
        do {
            int tmp = at_time;
            sc += eval(tr.begin(), x, y, facing, at_time);
            if (at_time == tmp)
                break;
        } while (at_time < _steps);
        //cout << "bot, ok" << endl;
//    return (fitness_t)sc;
        return (score_t)sc;
    }

    bool is_turn_left(builtin_action a) const {
        return a == id::turn_left;
    }
    bool is_turn_right(builtin_action a) const {
        return a == id::turn_right;
    }
    bool is_move_forward(builtin_action a) const {
        return a == id::move_forward;
    }

    int eval(sib_it it, int& x, int& y, Direction& facing, int& at_time) const {

        // std::cout << "ANT EVAL : " << combo_tree(it) << std::endl;

        // OC_ASSERT(is_action(*it) || is_builtin_action(*it));

        if (at_time >= _steps)
            return 0;
        if (*it == id::null_vertex) {
            return 0;
        }
        if (*it == id::action_success) {
            return 0;
        }
        if (*it == id::sequential_and) { //need to recurse
            int res = 0;
            for (sib_it sib = it.begin();sib != it.end();++sib)
                res += eval(sib, x, y, facing, at_time);
            return res;
        } else if (*it == id::action_if) {

            int xn = x, yn = y;
            //if-food-ahead
            switch (facing) {
            case east:
                xn = (x + 1) % ANT_X;
                break;
            case west:
                xn = (x - 1 + ANT_X) % ANT_X;
                break;
            case south:
                yn = (y + 1) % ANT_Y;
                break;
            case north:
                yn = (y - 1 + ANT_Y) % ANT_Y;
                break;
            }
            sib_it b1 = ++it.begin();
            sib_it b2 = ++(++it.begin());
            return eval(trail[yn][xn] != ' ' ? b1 : b2, x, y, facing, at_time);
        } else {
            OC_ASSERT(is_builtin_action(*it));
            ++at_time;
            builtin_action a = get_builtin_action(*it);
            if (is_move_forward(a)) { //move forward
                switch (facing) {
                case east :
                    x = (x + 1) % ANT_X;
                    break;
                case west :
                    x = (x - 1 + ANT_X) % ANT_X;
                    break;
                case south:
                    y = (y + 1) % ANT_Y;
                    break;
                case north:
                    y = (y - 1 + ANT_Y) % ANT_Y;
                    break;
                }
                if (trail[y][x] != ' ') {
                    trail[y][x] = ' ';
                    return 1;
                }
            } else if (is_turn_left(a)) { //rotate left
                turn_left(facing);
            } else if (is_turn_right(a)) { //rotate right
                turn_right(facing);
            } else {
                assert(false);
                ++at_time;
                //assert(is_reversal(*it));
                //reverse(facing);
            }
            return 0;
        }
    }

private:
    const int _steps;

};

struct AntFitnessEstimator : public AntFitnessFunction {
    AntFitnessEstimator(int steps = 600, int noise = 0)
            : AntFitnessFunction(steps), _noise(noise) { }
    result_type operator()(argument_type tr) const {
        int error = opencog::randGen().randint(_noise + 1) - _noise / 2;
        return AntFitnessFunction::operator()(tr) + error;
    }

private:
    const int _noise;
};


struct ant_score : public cscore_base
{
    ant_score() {}
    composite_score operator()(const combo_tree& tr) const {
        return composite_score (-1000 + aff(tr), 0, 0);
    }
private:
    AntFitnessFunction aff;
};

// @todo: it is probability not a good behavioral_score
struct ant_bscore : public bscore_base
{
    penalized_bscore operator()(const combo_tree& tr) const
    {
        penalized_bscore pbs;
        pbs.first.push_back(get_score(ant_score()(tr)));
        pbs.second = tr.size();

        return pbs;
    }

    behavioral_score best_possible_bscore() const
    {
        return {0.0};
    }
};

#endif
