/*
 * opencog/learning/moses/moses/ant_scoring.cc
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

#include "ant_scoring.h"

////////////////////////
// AntFitnessFunction //
////////////////////////

AntFitnessFunction::AntFitnessFunction(int steps)
    : _steps(steps) {}

void AntFitnessFunction::turn_left(Direction& d) {
    d = (Direction)(((int)d + 3) % 4);
}
void AntFitnessFunction::turn_right(Direction& d) {
    d = (Direction)(((int)d + 1) % 4);
}
void AntFitnessFunction::reverse(Direction& d) {
    d = (Direction)(((int)d + 2) % 4);
}

score_t AntFitnessFunction::operator()(const combo_tree& tr) const {
    char trail[ANT_Y][ANT_X+1];
    copy(init_trail[0], init_trail[0] + sizeof(init_trail), trail[0]);

    if (tr.empty())
        return MIN_FITNESS;
    int x = 0, y = 0;
    Direction facing = east;
    int at_time = 0;
    int sc = 0;
    do {
        int tmp = at_time;
        sc += eval(tr.begin(), x, y, facing, at_time, trail);
        if (at_time == tmp)
            break;
    } while (at_time < _steps);
    //cout << "bot, ok" << endl;
    //    return (fitness_t)sc;
    return (score_t)sc;
}

bool AntFitnessFunction::is_turn_left(builtin_action a) const {
    return a == id::turn_left;
}
bool AntFitnessFunction::is_turn_right(builtin_action a) const {
    return a == id::turn_right;
}
bool AntFitnessFunction::is_move_forward(builtin_action a) const {
    return a == id::move_forward;
}

int AntFitnessFunction::eval(sib_it it, int& x, int& y,
                             Direction& facing, int& at_time,
                             char trail[ANT_Y][ANT_X+1]) const {

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
            res += eval(sib, x, y, facing, at_time, trail);
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
        return eval(trail[yn][xn] != ' ' ? b1 : b2,
                    x, y, facing, at_time, trail);
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

/////////////////////////
// AntFitnessEstimator //
/////////////////////////

AntFitnessEstimator::AntFitnessEstimator(int steps, int noise)
    : AntFitnessFunction(steps), _noise(noise) {}
score_t AntFitnessEstimator::operator()(const combo_tree& tr) const {
    int error = opencog::randGen().randint(_noise + 1) - _noise / 2;
    return AntFitnessFunction::operator()(tr) + error;
}

////////////////
// ant_bscore //
////////////////

ant_bscore::ant_bscore(float simplicity_pressure)
    : _simplicity_pressure(simplicity_pressure) {}

behavioral_score ant_bscore::operator()(const combo_tree& tr) const
{
    behavioral_score bs;
    // @todo: It is not a good behavioral_score. Instead it should be
    // -1 for all pellet non-eaten
    bs.push_back(-89);
    bs.push_back(_aff(tr));

    return bs;
}

behavioral_score ant_bscore::best_possible_bscore() const
{
    return {0.0};
}
