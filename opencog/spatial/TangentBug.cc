/*
 * opencog/spatial/TangentBug.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Dan Zwell
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

#include <opencog/util/random.h>
#include <opencog/spatial/TangentBug.h>
#include <opencog/spatial/TB_ASSERT.h>

#include <deque>
#include <limits>
#include <utility>
#include <iostream>
#include <algorithm>
#include <boost/bind.hpp>
#include <opencog/util/RandGen.h>

using namespace opencog;
using namespace opencog::spatial;

/** ---------------------------------------------------------------------------
 * Protected functions
 * ----------------------------------------------------------------------------
 */
void TangentBug::init_vars()
{
    edge_following_recently_ended = false;
    curr_pos = TBPoint(0, 0);
    center = TBPoint(lsm->xDim() / 2, lsm->yDim() / 2);
    random_walk_randomness = 1;

    debug_look_along_ray = false;
    debug_wall_follow = false;
}

bool TangentBug::is_local_minimum(const TBPoint& next_point) const
{
    // True if we are getting closer to the goal with the next step (or if we
    // aren't moving).
    return (((goal - next_point).length() > (goal - curr_pos).length()) ||
            (next_point == curr_pos));
}

TBRay TangentBug::dir_closest_wall()
{

    double delta = PI / 24;
    TBPoint pt;

    // We look in circles, moving out concentrically.
    for (int distance_to_look = 1; distance_to_look <= SIGHT_DISTANCE; ++distance_to_look) {
        for (double theta = 0; theta < 2 * PI; theta += delta) {
            pt.first = curr_pos.first + static_cast<int>(distance_to_look * cos(theta));
            pt.second = curr_pos.second + static_cast<int>(distance_to_look * sin(theta));
            //logger().fine("TangentBug::dir_closest_wall(): before gridIllegal");
            if (lsm->gridIllegal(pt))
                // This point must be the closest wall.
                return (pt - curr_pos);
        }
    }

    TB_ASSERT(false); // This method has failed if this line is reached.
    return TBRay(); // Keep the compiler warnings at bay...
}

void TangentBug::cleanup_action_plan()
{
    logger().info("TangentBug - Shortening action plan. It has %d elem.",
                 calculatedPath.size());

    // Repeatedly ensure that the path is free between begin_range and end_range,
    // and erase the range (begin_range,end_range)
    CalculatedPath::iterator begin_range = calculatedPath.begin();
    CalculatedPath::iterator end_range = calculatedPath.begin();
    using boost::tuples::get;

    for (; begin_range != calculatedPath.end(); ++begin_range) {
        //Point pt = get<1>(*begin_range);
        end_range = begin_range;
#ifdef TB_ONLY_STRAIGHTEN_RANDOM_PATHS
        int status = get<2>(*begin_range);
        if (status != RANDOM_WALK)
            continue;
#endif
        while (end_range != calculatedPath.end() &&
                path_free(get<1>(*begin_range), get<1>(*end_range))) {
            ++end_range;
        }

        --end_range;
        if (begin_range != end_range) {
            TB_ASSERT(end_range != calculatedPath.end());
            TB_ASSERT(path_free(get<1>(*begin_range), get<1>(*end_range)));
            calculatedPath.erase(++begin_range, end_range);
            // begin_range was invalidated:
            begin_range = --end_range;
        }
    }
    //TB_ASSERT(! actionPlan.empty());

    // We would like to but cannot cannot TB_ASSERT that the path is free between
    // every two points because that's not necessarily true. We could TB_ASSERT
    // that there is some free straight path from A to B, but not that the
    // (main) path from A to B is free.
    logger().info(" TangentBug - Shortening action plan complete. It has %d elem.",
                 calculatedPath.size());
}

bool TangentBug::path_free_to_goal()
{

    if ((goal - curr_pos).length() <= SIGHT_DISTANCE) {
        return path_free(curr_pos, goal);
    }
    return path_free(curr_pos, goal - curr_pos, SIGHT_DISTANCE);
}

// Perform no checks, but go to a point:
bool TangentBug::_move(const TBRay& r, double distance)
{
    return _move(curr_pos + r.normalize() * distance);
}

bool TangentBug::_move(const TBPoint& dest)
{
    logger().debug("TangentBug - _move() called.");
    TB_ASSERT(movement != UNSET);

    if (edge_following_recently_ended) {
        edge_following_recently_ended = false;
    }

    prev_pos = curr_pos;
    curr_pos = TBPoint(dest);
    logger().debug("TangentBug - Moved to: (%d, %d).", curr_pos.first, curr_pos.second);

    trace_path(prev_pos, curr_pos, TRACE_PURPLE);

    aux_map[prev_pos].erase(CURR_POS);
    aux_map[curr_pos].insert(CURR_POS);
    print(); // Draw the map on the screen.

    int status_code = movement;
    calculatedPath.push_back(boost::tuples::make_tuple(lsm->unsnap(curr_pos),
                             curr_pos,
                             status_code));
    if (tb_timeout()) {
        return false;
    }

#ifndef NDEBUG
    std::pair<double, double> point_pair = boost::tuples::get<0>(calculatedPath.back());
    TB_ASSERT(curr_pos == lsm->snap(point_pair));
#endif

    //logger().fine("TangentBug::_move(): before gridIllegal");
    TB_ASSERT(! lsm->gridIllegal(curr_pos));
    return true;
}

bool TangentBug::tb_timeout()
{

#ifndef TB_NO_TIMEOUT
    TB_ASSERT(movement != UNSET);
    if (movement == TO_GOAL) {
        return false; // No conceivable reason to time out.
    }

    {
        unsigned int max_number_steps = 3000;
        if (calculatedPath.size() > max_number_steps) {
            logger().warn(
                         "TangentBug - Algorithm timed out in the worst possible "
                         "way. The relatively nice timeout algorithm was not triggered, but "
                         "instead the maximum allowed number of moves was reached. This is "
                         "surely because you are on a very large map. Either the "
                         "max_number_steps is too small, or M and N are too small, and "
                         "cycles are going unnoticed.");
            return true;
        }
    }

    // The basic idea is that we want to time out if we are "not really going
    // anywhere". We measure that by comparing the minimum bounding rectangle
    // of the positions we have occupied in the last N moves with the
    // positions we have occupied in the last M moves. If they are very
    // similar, we are not really going anywhere new, and we should time out.
    //
    // For maximum efficacy, we would pick M and N large--after all, if the
    // bug is trapped in a map it can't figure out, and it is moving in a
    // pattern that repeats every 120 moves or so, this heuristic will not be
    // triggered unless M and N are >= 120.
    //
    // However, if M and N are large, we must wait at least max(M,N) moves
    // before we can tell that we have timed out.

    // TWEAK ME:
    unsigned int M = 200;
    unsigned int N = 500;
    TB_ASSERT(M < N);
    if (calculatedPath.size() < N) {
        return false; // We can't work in these conditions!
    }

    // Coordinates of minimum bounding rectangle in the last M moves:
    coord lowest_m = std::numeric_limits<coord>::max(),
                     leftmost_m   = std::numeric_limits<coord>::max(),
                                    highest_m    = std::numeric_limits<coord>::min(),
                                                   rightmost_m  = std::numeric_limits<coord>::min();

    CalculatedPath::iterator itr = calculatedPath.end();
    for (unsigned int i = 0; i < M; ++i) {
        --itr;
        const TBPoint& p = boost::tuples::get<1>(*itr);
        if (p.first  < leftmost_m)  leftmost_m  = p.first;
        if (p.first  > rightmost_m) rightmost_m = p.first;
        if (p.second < lowest_m)    lowest_m    = p.second;
        if (p.second > highest_m)   highest_m   = p.second;
    }
    TB_ASSERT(leftmost_m <= rightmost_m);
    TB_ASSERT(lowest_m <= highest_m);

    // Coordinates of minimum bounding rectangle in the last N moves:
    coord lowest_n = lowest_m,
                     leftmost_n   = leftmost_m,
                                    highest_n    = highest_m,
                                                   rightmost_n  = rightmost_m;
    // Iterate through the list of the remaining N-M points to consider:
    for (unsigned int i = 0; i < (N - M); ++i) {
        --itr;
        const TBPoint& p = boost::tuples::get<1>(*itr);
        if (p.first  < leftmost_n)  leftmost_n  = p.first;
        if (p.first  > rightmost_n) rightmost_n = p.first;
        if (p.second < lowest_n)    lowest_n    = p.second;
        if (p.second > highest_n)   highest_n   = p.second;
    }
    TB_ASSERT(leftmost_n <= rightmost_n);
    TB_ASSERT(lowest_n <= highest_n);

    // If the minimum bounding rectangle around the M last points does not
    // differ significantly from that around the N last points, return true, the
    // algorithm has timed out. Calculate similarity by percent of area shared.

    /* // This isn't necessary if one rectangle is completely enclosed
       coord lowest_overlapping = max(lowest_m, lowest_n),
       leftmost_overlapping   = max(leftmost_m, leftmost_n),
       highest_overlapping    = min(highest_m, highest_n),
       rightmost_overlapping  = min(rightmost_m, rightmost_n);
    // If there is no overlap:
    if (leftmost_overlapping > rightmost_overlapping ||
    lowest_overlapping > highest_overlapping)
    return false;
    */

    // The diagonals of the minimum bounding rectangles:
    double diagonal_m = sqrt(pow((double)rightmost_m - leftmost_m, 2) +
                             pow((double)highest_m - lowest_m , 2));
    double diagonal_n = sqrt(pow((double)rightmost_n - leftmost_n, 2) +
                             pow((double)highest_n - lowest_n , 2));
    double diagonal_epsilon = sqrt(pow(std::numeric_limits<double>::min(), 2) +
                                   pow(std::numeric_limits<double>::min(), 2));

    TB_ASSERT(diagonal_n >= diagonal_m - diagonal_epsilon);
    if (diagonal_m <= diagonal_epsilon) {
        logger().error("TangentBug - Bug is trapped in some sort of corner. Failed.");
        printLocalSpaceMap();
        return true;
    }

    double similarity = diagonal_m / diagonal_n;

    TB_ASSERT(0 <= similarity && similarity <= 1);

    // TWEAK ME: what similarity is grounds for failure?
    if (similarity >= 0.85) {
        logger().error("TangentBug - The area the bug has occupied in the last %d steps does not differ significantly from the area the bug has occupied in the last %d steps. Terminating.", M, N);
        printLocalSpaceMap();
        return true;
    } else return false;

#else
    return false;
#endif // ifndef TB_NO_TIMEOUT

}

bool TangentBug::follow_edge()
{

    // Used to tell when we have gone in a circle:
    std::vector<TBPoint> first_few_points;
    std::deque <TBPoint> most_recent_points;
    first_few_points.push_back(curr_pos);

    // d_followed is the shortest distance between the sensed boundary and
    // the goal.
    // d_reach is the shortest distance between blocking obstacle and goal
    // (or my distance to goal if no blocking obstacle visible)--that is,
    // if we have a clear line of sight to the goal
    //   the distance to the goal
    // else
    //   there is an obstacle between us and the goal. on that obstacle,
    //   the shortest distance to the goal that we can see.
    double d_followed = (curr_pos - goal).length();
    double d_reach = d_followed;
    //d_followed = d_reach = (curr_pos - goal).length();

    TBRay moving_direction1;
    get_wall_tangent(LEFT, moving_direction1);
    TBRay moving_direction2;
    get_wall_tangent(RIGHT, moving_direction2);

    // Which of these directions is more aligned with the direction we were just
    // moving? Maximize the dot product with the direction we are moving...
    TBRay old_dir = curr_pos - prev_pos;
    int wall_follow_dir;

    if (old_dir * moving_direction1 > old_dir * moving_direction2) {
        wall_follow_dir = LEFT;
    } else {
        wall_follow_dir = RIGHT;
    }

    // Wall following conditions:
    while ((d_reach >= (d_followed - std::numeric_limits<double>::epsilon())) &&
            (goal != curr_pos)) {

        // *************** Pick a direction to move: ****************************
        TBRay moving_direction;
        bool trapped = ! get_wall_tangent(wall_follow_dir, moving_direction);

        if (trapped) {
            trace_path(curr_pos, curr_pos + moving_direction);
            logger().error("TangentBug - It seems to have become trapped.");
            printLocalSpaceMap();
            //print();
            return false;
        }

        // We have a direction to move. Move there:
        if (! _move(moving_direction, FOLLOW_STEP_DISTANCE)) {
            return false;
        }

        // ******** Evaluate whether we have gone in a circle: ***************

        // Update the two position lists:
        if (first_few_points.size() < 5) {
            first_few_points.push_back(curr_pos);
        } else {
            most_recent_points.push_back(curr_pos);
            if (most_recent_points.size() > 5) {
                most_recent_points.pop_front();
            }

            // We are going in circles if 3 of the 5 point pairs (first point
            // and most recent point-4, second point and most recent point - 3,
            // ..., fifth point, most recent point) match in that:
            // 1) There is no obstacle between them
            // 2) they are within a step of each other.
            int matching_points = 0;
            for (int i = 0; i < 5; ++i) {
                if (((first_few_points[i] - most_recent_points[i]).length()) <=
                        (1.5 * FOLLOW_STEP_DISTANCE) &&
                        path_free(first_few_points[i], most_recent_points[i])) {
                    matching_points++;
                }
            }

            if (matching_points >= 3) {
                logger().info("TangentBug - Appears to be chasing our tail. Obstacle is unreachable.");
                return false;
            }
        }

        // ************ Update d_reach and d_follow **********************
        // d_followed: closest distance on wall to goal
        d_followed = std::min(d_followed,
                              closest_distance_on_obstacle_to_goal(dir_closest_wall()));
        TB_ASSERT(look_along_ray(curr_pos, dir_closest_wall(), SIGHT_DISTANCE).collided);

        // When moving is discrete steps greater than 2, it is possible to step
        // over or past the goal without breaking the edge following
        // condition. This is a modification to the TangentBug algorithm to render
        // this impossible.
        if (path_free_to_goal()) {
            if ((goal - curr_pos).length() <= SIGHT_DISTANCE) {

                // Break edge following, because the goal is in sight and reachable.
                d_reach = 0;
            } else {

                // This is what the pure TB algorithm says to do:
                d_reach = (goal - curr_pos).length();
            }
        } else {
            d_reach = closest_distance_on_obstacle_to_goal(goal - curr_pos);
        }

        if (debug_wall_follow) {
            std::cerr << "setting d_reach to: " << d_reach << std::endl;
            std::cerr << "setting d_followed to: " << d_followed << std::endl;
            std::cerr << "curr_pos: " << curr_pos << std::endl;
            std::cerr << "goal: " << goal << std::endl;
            std::cerr << "path to goal is free? " << path_free_to_goal() << std::endl;
        }

    } // while wall following conditions hold
    return true;
}

double TangentBug::closest_distance_on_obstacle_to_goal(TBRay dir_to_object)
{

    // Scan left, scan right
    double deltas[] = { PI / 30, -PI / 30 };
    double best_distance; { // = std::numeric_limits<double>::max();
        TBPoint best_point =
            look_along_ray(curr_pos, dir_to_object, SIGHT_DISTANCE).last_point;
        best_distance = (best_point - goal).length();
    }

    for (double delta : deltas) {
        look_info prev_glance = look_along_ray(curr_pos,
                                                dir_to_object,
                                                SIGHT_DISTANCE);
        TB_ASSERT(prev_glance.collided);

        TBPoint prev_point = prev_glance.last_point;
        TBRay r(dir_to_object);

        for (double theta = 0; std::fabs(theta) < PI; theta += delta) {
            // Rotate r theta degrees
            double _x = cos(delta) * r.x - sin(delta) * r.y;
            double _y = sin(delta) * r.x + cos(delta) * r.y;
            r.x = _x, r.y = _y;

            // Find point on wall in direction r
            look_info vision = look_along_ray(curr_pos, r, SIGHT_DISTANCE);
            if (!vision.collided ||
                    (vision.last_point - prev_point).length() > MAX_CONTIG_DISTANCE) {
                break;
            }

            best_distance = std::min((vision.last_point - goal).length(), best_distance);
            prev_point = vision.last_point;
        }
    }
    return best_distance;
}

std::vector<look_info> TangentBug::get_obstacle_endpoints()
{

    std::vector<look_info> look_dests = get_visible_destinations();
    std::vector<look_info> endpoints;
    look_info prev_look = look_dests.back();

    // For each point:
    //
    // If this point collided and the last one didn't, mark this point as an
    // endpoint. If this point did not collide but the next one does, mark the
    // next point (that is, if this point didn't collide but the previous point
    // did, mark the previous point). If neither is true and this point did not
    // hit, go to the next point.
    //
    // If neither is true and this point did collide, mark it as an endpoint if
    // it is farther than a certain distance from the previous point. In that
    // case, also mark the previous point.
    //
    // To deal with the fact that these points are circular, the last shall be
    // first and the first shall be last:

    // Can I safely use reference (e.g., look_info& this_look) if I am aggregating them in a container:
    for (look_info this_look : look_dests) {
        if (! this_look.collided) {
            if (prev_look.collided) {
                endpoints.push_back(prev_look);
            }
        } else {
            // This point collided
            if (! prev_look.collided) {
                endpoints.push_back(this_look);
            } else {
                // Is there discontinuity based on distance?
                if ((this_look.last_point - prev_look.last_point).length() > MAX_CONTIG_DISTANCE) {
                    endpoints.push_back(prev_look);
                    endpoints.push_back(this_look);
                }
            }
        }
        prev_look = this_look;
    }
    return endpoints;
}

std::vector<look_info> TangentBug::get_visible_destinations2()
{
    logger().warn("TangentBug - get_visible_destination2 is not implemented. Returning an empty look_info struct.");

    // TODO: the idea is to represent the area around me curr_pos as a
    // continuous interval of 360 degrees, and I do not look all the way to the
    // end of my vision (at first), because that would be needlessly
    // computationally intensive--say, at the end of my vision's range, I need
    // 200 rays to see enough things to suitably many occupied points to judge
    // breaks in continuity. However, during the first 5 squares in every
    // direction, 20 rays would be sufficient. So at first, I can send out 20
    // rays, and mark intervals (of the 360 degrees) where the rays collided
    // (within the first 5 squares) as being known. Then I would send out, say,
    // 40 rays (this actually must be proportional to r^2) that would travel
    // from r=5 to r=10 (in unknown angular intervals) of the surrounding
    // space. These similarly would mark intervals as known when they hit. This
    // would continue out to r=SIGHT_DISTANCE or until the entire area from 0 to
    // 360 was known.
    return std::vector<look_info>();
}

std::vector<look_info> TangentBug::get_visible_destinations()
{

    std::vector<look_info> visible_points;
    TBRay gaze((double)SIGHT_DISTANCE, 0.f);

    // We want to see 300 points, which is an arbitrary number.
    //double theta_delta = PI / 150;
    double theta_delta = PI / 150;
    double s = std::sin(theta_delta);
    double c = std::cos(theta_delta);

    // Theta is just a counter.
    for (double theta = 0; theta < 2 * PI; theta += theta_delta) {
        double _x = (c * gaze.x) - (s * gaze.y);
        double _y = (s * gaze.x) + (c * gaze.y);
        gaze.x = _x;
        gaze.y = _y;
        visible_points.push_back(look_along_ray(curr_pos, gaze, SIGHT_DISTANCE));
    }
    return visible_points;
}

void TangentBug::_place_pet(const TBPoint& pt)
{
    // check not an illegal position
    //logger().fine("TangentBug::_place_pet(): before gridIllegal");
    TB_ASSERT(!lsm->gridIllegal(pt));
    curr_pos = pt;
    aux_map[pt].insert(CURR_POS);
}

void TangentBug::_place_goal(const TBPoint& pt, bool allow_enclosed)
{
    if (!allow_enclosed) {
        //logger().fine("TangentBug::_place_goal(): before gridIllegal");
        TB_ASSERT(! lsm->gridIllegal(pt));
    }

    goal = pt;
    aux_map[pt].insert(GOAL);
}

bool TangentBug::get_wall_tangent(int right_or_left, TBRay& tangent)
{
    // Initial stab at a direction--we assume this is blocked.
    tangent  = dir_closest_wall().normalize() * FOLLOW_STEP_DISTANCE;
    double theta_delta = PI / 24;

    if (right_or_left == RIGHT) {
        theta_delta = -theta_delta;
    }

    double rotated_degrees = 0;
    double s = sin(theta_delta), c = cos(theta_delta);

    // Until our step vector leads to a free path
    do {
        // Rotate tangent theta_delta degrees around the z-axis
        double _x = c * tangent.x - s * tangent.y;
        double _y = s * tangent.x + c * tangent.y;
        tangent.x = _x, tangent.y = _y;

        if (std::fabs(rotated_degrees) > 2*PI) {
            trace_path(curr_pos, curr_pos + tangent, TRACE_RED); print();
            logger().error("TangentBug - Could not find a tangent to the wall. May be trapped.");
            printLocalSpaceMap();
            return false;
        }
        rotated_degrees += theta_delta;

    } while (! path_free(curr_pos, tangent, FOLLOW_STEP_DISTANCE));

    TB_ASSERT(path_free(curr_pos, tangent, FOLLOW_STEP_DISTANCE));
    return true;
}

bool TangentBug::path_free(const TBPoint& pt1,
                           const TBPoint& pt2) const
{
    if (pt1 == pt2) {
        //logger().fine("TangentBug::path_free(): before gridIllegal");
        return ! lsm->gridIllegal(pt1);
    }

    TBRay r = pt2 - pt1;
    return path_free(pt1, r, r.length());
}

bool TangentBug::path_free(const TBPoint& pt1,
                           const TBRay& direction,
                           double distance) const
{
    return !(look_along_ray(pt1, direction, distance).collided);
}

bool TangentBug::reeval_subgoal(look_info &new_subgoal)
{

    if (path_free_to_goal()) {
        movement = TO_GOAL;
        look_info look(curr_pos, goal);
        look.orig_direction = goal - curr_pos;
        logger().debug("TangentBug - Path to goal is free.");
        new_subgoal = look;
        return true;
    }

    movement = NORMAL;
    std::vector<look_info> endpoints = get_obstacle_endpoints();

    //logger().debug("TangentBug::reeval_subgoal(): endpoints' size = %u", endpoints.size());

    std::vector<look_info>::iterator new_end;

    // NOTE: The following paragraph my not be true, it has been tweaked lots.
    // If curr_pos is ever a point if discontinuity, it will be found to be the
    // optimal subgoal. That is a fact, based on the rules of the TangentBug
    // algorithm. (d(curr_pos, curr_pos)+d(curr_pos, goal) is minimized.) We do
    // not want this, so we do not allow curr_pos to be considered as a
    // potential subgoal.

    // If the bug has just finished edge following, try to prevent it from
    // starting again. Do this by filtering all points that are very close to our
    // current position.
    if (edge_following_recently_ended) {
        new_end = remove_if(endpoints.begin(), endpoints.end(),
                            boost::bind(&TangentBug::closer_than_n_pixels, this, _1, 3, false));
    }

    // Otherwise, just filter the points that are very close and not in the
    // general direction of the goal.
    else {
        new_end = remove_if(endpoints.begin(), endpoints.end(),
                            boost::bind(&TangentBug::closer_than_n_pixels, this, _1, 3, true));
    }

    endpoints.erase(new_end, endpoints.end());
    for (look_info &look : endpoints) {
        aux_map[look.last_point_before_hit].insert(TRACE_CYAN);
    }

    print(); //getch();
    for(look_info &look : endpoints) {
        aux_map[look.last_point_before_hit].erase(TRACE_CYAN);
    }

    double best_distance = std::numeric_limits<double>::max();
#ifndef TB_NO_RANDOM_WALK
    if (endpoints.empty()) {
        // We see no points of discontinuity, and thus can't move. Most likely, we
        // are just in an enclosed space with a very small opening.
        //random_walk(2);
        if (! random_walk(2)) {
            return false;
        }
        return reeval_subgoal(new_subgoal);
    }
#else
    TB_ASSERT(! endpoints.empty());
#endif

    look_info best_point;
    double heuristic_distance;
    for (look_info& look : endpoints) {
        TBPoint * pt = &look.last_point_before_hit;
        heuristic_distance = (curr_pos - *pt).length() + (*pt - goal).length();
        if (heuristic_distance < best_distance) {
            best_distance = heuristic_distance;
            best_point = look;
        }
    }
    //logger().fine("TangentBug::reeval_subgoal(): before gridIllegal");
    TB_ASSERT(! lsm->gridIllegal(best_point.last_point_before_hit));
    new_subgoal = best_point;
    return true;
}


/* ----------------------------------------------------------------------------
 * Public functions
 * ----------------------------------------------------------------------------
 */
TangentBug::TangentBug(const Map& lsm, CalculatedPath& calculatedPath) :
        lsm(&lsm), calculatedPath(calculatedPath), rng(randGen())
{
    init_vars();
}

TangentBug::~TangentBug()
{
}

const TBPoint& TangentBug::getCenter() const
{
    return const_cast<const TBPoint&>(center);
}

const TBPoint& TangentBug::getGoal() const
{
    return const_cast<const TBPoint&>(goal);
}

const TBPoint& TangentBug::getCurrPos() const
{
    return const_cast<const TBPoint&>(curr_pos);
}

bool TangentBug::seek_goal()
{

    movement = UNSET;
    print();

    while (curr_pos != goal) {
        logger().debug("TangentBug - pet pos (%d, %d), goal (%d, %d)",
                     curr_pos.first, curr_pos.second, goal.first, goal.second);

        look_info subgoal;
        if (!reeval_subgoal(subgoal)) {
            logger().debug("TangentBug - Cannot reeval subgoal. Clean path.");
            cleanup_action_plan();
            return false;
        }
        //logger().fine("TangentBug::seek_goal(): before gridIllegal");
        TB_ASSERT(!lsm->gridIllegal(subgoal.last_point_before_hit));

        if (is_local_minimum(subgoal.last_point_before_hit)) {
            // We are in a local minimum of some sort, and we need to edge follow.

#ifndef TB_NO_RANDOM_WALK
            if (edge_following_recently_ended) {
                // We will likely enter an infinite loop unless we do something.
                //random_walk(3);
                if (! random_walk(3)) {
                    logger().error(
                                 "TangentBug - Random walk to scape failed! Clean path.");
                    cleanup_action_plan();
                    return false;
                }
                continue;
            }
#else
            // Infinite loops are likely here, and the following TB_ASSERTion
            // is not enough to catch them.
            TB_ASSERT(! edge_following_recently_ended);
#endif

            logger().debug("TangentBug - Start EDGE FOLLOWING");
            movement = FOLLOWING;
            if (!follow_edge()) {
                logger().error("TangentBug - EDGE follow failed! Clean path.");
                printLocalSpaceMap();
                cleanup_action_plan();

                return false;
            }

            // Keep state information...
            edge_following_recently_ended = true;
            logger().debug("TangentBug - End EDGE FOLLOWING");
            movement = NORMAL;
        } else {
            switch (movement) {

            case TO_GOAL :
                // In this case, there is no lost directional information if we just
                // move toward a point--the directional information was created based
                // on the point.
                if ((subgoal.last_point_before_hit - curr_pos).length() > MAX_STEP_DISTANCE) {
                    if (!_move(subgoal.orig_direction, MAX_STEP_DISTANCE)) {
                        logger().error("TangentBug - Cannot move MAX_STEP_DIST. Clean path.");
                        printLocalSpaceMap();
                        cleanup_action_plan();
                        return false;
                    }
                    //logger().fine("TangentBug::seek_goal(): TO_GOAL 1: before gridIllegal");
                    TB_ASSERT(! lsm->gridIllegal(curr_pos));
                } else {
                    if (!_move(subgoal.last_point_before_hit)) {
                        logger().error("TangentBug - Cannot move last_point_before_hit. Clean path.");
                        printLocalSpaceMap();
                        cleanup_action_plan();
                        return false;
                    }
                    //logger().fine("TangentBug::seek_goal(): TO_GOAL 2: before gridIllegal");
                    TB_ASSERT(! lsm->gridIllegal(curr_pos));
                }
                break;

            case NORMAL :
                if ((subgoal.last_point_before_hit - curr_pos).length() > STEP_DISTANCE) {
                    //logger().fine("TangentBug::seek_goal(): NORMAL 1: before gridIllegal");
                    TB_ASSERT(!lsm->gridIllegal(subgoal.last_point_before_hit));

                    //debug_look_along_ray = true;
                    TB_ASSERT(path_free(curr_pos, subgoal.orig_direction, STEP_DISTANCE));

                    //aux_map[curr_pos + subgoal.orig_direction.normalize()*STEP_DISTANCE].insert(TRACE_RED);
                    //print();

                    //logger().fine("TangentBug::seek_goal(): NORMAL 2: before gridIllegal");
                    TB_ASSERT(! lsm->gridIllegal(curr_pos + subgoal.orig_direction.normalize()*STEP_DISTANCE));

                    if (!_move(subgoal.orig_direction, STEP_DISTANCE)) {
                        logger().error("TangentBug - Cannot move STEP_DISTANCE. Clean path.");
                        printLocalSpaceMap();
                        cleanup_action_plan();
                        return false;
                    }
                }

                else {
                    if (! _move(subgoal.last_point_before_hit)) {
                        logger().error("TangentBug - Cannot move last_point_before_hit 2. Clean path.");
                        printLocalSpaceMap();
                        cleanup_action_plan();
                        return false;
                    }
                    //logger().fine("TangentBug::seek_goal(): NORMAL 3: before gridIllegal");
                    TB_ASSERT(! lsm->gridIllegal(curr_pos));
                }
                break;

                // The following isn't used, because the following method calls _move directly:
                //
                //   case FOLLOWING :
                //   if (! _move(subgoal))    return false;
                //   break;
            default:
                logger().warn("TangentBug - Only TO_GOAL and NORMAL moves are treated.");
                break;
            }
        }
    }
    cleanup_action_plan();
    return true;
}

look_info TangentBug::look_along_ray(
    const TBPoint& initial_point,
    const TBRay& _direction,
    double distance) const
{
    if (distance < 0) {
        distance = SIGHT_DISTANCE;
    }
    TBRay r = _direction.normalize() * distance;

    look_info ret_val;
    ret_val.orig_direction = _direction;

    // This does not guarantee that last_point_before_hit is actually free.
    ret_val.last_point_before_hit = initial_point;

    //logger().fine("TangentBug::look_along_ray(): initial_point: before gridIllegal");
    if (lsm->gridIllegal(initial_point)) {
        logger().warn("TangentBug - Returning a struct where last_point_before_hit is in fact not before the hit, because the first point is occupied or out of bounds.");
        ret_val.last_point_before_hit = initial_point;
        ret_val.last_point = initial_point;
        ret_val.collided = true;
        return ret_val;
    }

    // This is Bresenham's line drawing algorithm, modified to deal with
    // floating point (yes, that ruins some of the efficiency) because we
    // really need to deal with actual vectors, and also modified to fill in
    // every cell that a ray passes through (not just one per column or
    // row). See http://www.gamedev.net/reference/articles/article1275.asp for a
    // similar but less complex implementation. This also borrows ideas from the
    // wikipedia article on this topic.
    double delta_x = std::fabs(r.x);
    double delta_y = std::fabs(r.y);
    double error_delta;

    // If this is so, we'll need to loop over
    bool steep = (delta_x < delta_y);

    // y, and increment (or not) x. We have two directions of movement--"main" and
    // "otro". The main direction is the one that moves faster, while "incr" is
    // sometimes incremented and sometimes not. If (steep), main is the y
    // coordinate.
    coord main_coord;
    coord otro_coord;
    int main_incr;
    int otro_incr;

    // This row / column is NOT drawn, it is beyond the last point:
    coord main_end;
    coord otro_end;

    // We want to be able to refer to x and y by name, in addition to thinking
    // of them as the main coordinate and the otro coordinate (or vice-versa,
    // which is the problem)
    const coord *px;
    const coord *py;
    const coord *px_end;
    const coord *py_end;

    if (steep) {
        // y:
        main_coord = initial_point.second;
        main_incr  = (r.y > 0) ? 1 : -1; // The main coordinate will move up or down?
        main_end   = (initial_point + r).second + main_incr;
        py_end = &main_end;
        py = &main_coord;
        // x:
        otro_coord = initial_point.first;
        otro_incr  = (r.x > 0) ? 1 : -1;
        otro_end   = (initial_point + r).first + otro_incr;
        px_end = &otro_end;
        px = &otro_coord;

        error_delta = delta_x / delta_y;
    } else {
        // x:
        main_coord = initial_point.first;
        main_incr  = (r.x > 0) ? 1 : -1; // The main coordinate will move left or right?
        main_end   = (initial_point + r).first + main_incr;
        px_end = &main_end;
        px = &main_coord;
        // y:
        otro_coord = initial_point.second;
        otro_incr  = (r.y > 0) ? 1 : -1;
        otro_end   = (initial_point + r).second + otro_incr;
        py_end = &otro_end;
        py = &otro_coord;

        error_delta = delta_y / delta_x;
    }
    double error = error_delta / 2;

    const coord &x_end = *px_end;
    const coord &y_end = *py_end;
    const coord &x = *px;
    const coord &y = *py;

    /*
           fprintf(stderr, "main_coord: %i\n", main_coord);
           fprintf(stderr, "otro_coord: %i\n\n", otro_coord);
           fprintf(stderr, "main_end: %i\n", main_end);
           fprintf(stderr, "otro_end: %i\n", otro_end);
           fprintf(stderr, "error_delta: %0.2f\n", error_delta);
           fprintf(stderr, "main_incr: %i\n", main_incr);
           fprintf(stderr, "otro_incr: %i\n", otro_incr);
           std::cerr << "direction: " <<  r << std::endl;
           */


    // We do not know whether we are incrementing x or y, so x and y are
    // referred to by the handles main_coord and otro_coord. That is how they
    // are modified--"x" and "y" are const references.
    if (debug_look_along_ray) {
        std::cerr << "---------- look_along_ray():" << std::endl;
        std::cerr << "initial_point is " << initial_point << std::endl;
        std::cerr << "the last point is (should be): " <<
                  (initial_point + r) << std::endl;
        std::cerr << "r is " << r << std::endl;
        std::cerr << "steep is " << (steep ? "true" : "false") << std::endl;
        std::cerr << "the end() row / column are: " <<
                  TBPoint(x_end, y_end) << std::endl;
        fprintf(stderr, "error_delta is %g\n", error_delta);
    }

    while (x != x_end && y != y_end) {
        //-------- do stuff with point
        ret_val.last_point = TBPoint(x, y);
        if (debug_look_along_ray) {
            std::cerr << "processing point " << ret_val.last_point << std::endl;
            fprintf(stderr, "error is %g: ", error);
        }

        // If we are off the map, or the grid here is occupied:
        //logger().fine("TangentBug::look_along_ray(): ret_val.last_point (%u,%u): before gridIllegal", x, y);
        if (lsm->gridIllegal(ret_val.last_point)) {
            ret_val.collided = true;

            // debug:
            if (debug_look_along_ray) {
                const_cast<TangentBug*>(this)->
                aux_map[ret_val.last_point].insert(TRACE_RED);
            }
            return ret_val;
        } else {
            ret_val.last_point_before_hit = ret_val.last_point;
        }

        if (error >= 0.5f) {
            error -= (1 + error_delta);
            // Move over one position in otro_incr, but move back (decrement) one
            // position in main_incr so that we are forced to deal with this column
            // (or row) twice and color both squares. This is why we subtracted more
            // than 1 from the error in the line above.
            otro_coord += otro_incr;
            main_coord -= main_incr;
        }

        error += error_delta;
        main_coord += main_incr;
    }

    if (debug_look_along_ray) {
        std::cerr << "the loop broke because " << (x == x_end ? "x == x_end" :
                  (y == y_end ? "y == y_end" :
                   "ERROR, loop shouldn't have broken")) <<
                  std::endl;
        std::cerr << "ret_val.last_point is " << ret_val.last_point << std::endl;
        std::cerr << "the next point, had the loop not broken, would be " <<
                  TBPoint(x, y) << std::endl;
    }

    // We have reached the end and no point has collided.
    TB_ASSERT(ret_val.last_point == ret_val.last_point_before_hit);
    TB_ASSERT(ret_val.last_point == initial_point + r);

    // TODO: check memory leak for *px *py and other
    return ret_val;
}

void TangentBug::test_look_along_ray() const
{
#ifndef NDEBUG
    for (int i = 0;i < 1000;++i) {
        double lenx = rng.randDoubleOneExcluded() * 100 * rng.randPositiveNegative();
        double leny = rng.randDoubleOneExcluded() * 100 * rng.randPositiveNegative();
        TBRay dir(lenx, leny);
        look_along_ray(curr_pos, dir, rng.randDoubleOneExcluded()*10);
    }
#endif
}

void TangentBug::place_pet(spatial::Distance x, spatial::Distance y)
{
    _place_pet(lsm->snap(spatial::Point(x, y)));
    logger().debug("TangentBug - Pet Coord (%.2f, %.2f), MapCoord (%d, %d).", x, y, curr_pos.first, curr_pos.second);
}

void TangentBug::place_goal(spatial::Distance x, spatial::Distance y)
{
    _place_goal(lsm->snap(spatial::Point(x, y)), true);
    logger().debug("TangentBug - Goal Coord (%.2f, %.2f), MapCoord (%d, %d).", x, y, goal.first, goal.second);
}

void TangentBug::trace_path2(const TBPoint& pt1,
                             const TBPoint& pt2)
{
}

void TangentBug::trace_path(const TBPoint &pt1,
                            const TBPoint &pt2,
                            int val)
{
#ifdef TB_PRINT_NCURSES
    // This method is not rigorous about coloring every single point--it may
    // appear perforated.
    TBRay direction = pt2 - pt1;
    double length = direction.length();
    point_map::iterator hash_itr;
    for (double itr = 0; itr < (int)length; itr += .1) {
        TBPoint r = (pt1 + (direction * (itr / length)));
        aux_map[pt1 + (direction * (itr / length))].insert(val);
    }
    aux_map[pt2].insert(val);
#endif
}

void TangentBug::untrace_path(const TBPoint &pt1,
                              const TBPoint &pt2,
                              int val)
{
#ifdef TB_PRINT_NCURSES
    TBRay direction = pt2 - pt1;
    double length = direction.length();
    point_map::iterator hash_itr;
    for (double itr = 0; itr < (int)length; itr += .1) {
        TBPoint r = (pt1 + (direction * (itr / length)));
        aux_map[pt1 + (direction * (itr / length))].erase(val);
    }
    aux_map[pt2].erase(val);
#endif
}

bool TangentBug::place_pet_randomly(const TBPoint& prob_center,
                                    int max_retries)
{
    if (max_retries < 0) {
        logger().error("TangentBug - Maximum number of retries to place the pet in an unoccupied space failed. Perhaps the grid is too full of obstacles?");
        printLocalSpaceMap();
        TB_ASSERT(max_retries >= 0);
        return false;
    }

    int x = gaussian_rand<unsigned>(prob_center.first, 25, rng);
    int y = gaussian_rand<unsigned>(prob_center.second, 25, rng);

    TBPoint pt(x, y);
    //logger().fine("TangentBug::place_pet_randomly(): before gridIllegal");
    if (! lsm->gridIllegal(pt)) {
        _place_pet(pt);
        return true;
    }
    return place_pet_randomly(prob_center, max_retries - 1);
}

bool TangentBug::place_goal_randomly(const TBPoint& prob_center,
                                     bool allow_enclosed, int max_retries)
{

    if (max_retries < 0) {
        logger().error("TangentBug - Maximum number of retries to place the goal in an unoccupied space failed. Perhaps the grid is too full of obstacles?");
        printLocalSpaceMap();
        TB_ASSERT(max_retries >= 0);
        return false;
    }

    int x = gaussian_rand<unsigned>(prob_center.first, 300, rng);
    int y = gaussian_rand<unsigned>(prob_center.second, 200, rng);

    //logger().fine("TangentBug::place_pet_randomly(): before gridIllegal");
    TBPoint pt(x, y);
    if (lsm->coordinatesAreOnGrid(x, y) &&
            (allow_enclosed || ! lsm->gridIllegal(pt))) {
        _place_goal(pt, allow_enclosed);
        return true;
    }
    return place_goal_randomly(prob_center, allow_enclosed, max_retries - 1);
}

void TangentBug::print() const
{
#ifdef TB_PRINT_NCURSES
    clear();

    // x and y are cartesian coordinates, while screen_x and screen_y are
    // ncurses coordinates.
#ifdef TB_PRINT_CENTERED_AT_CURR_POS
    // width and height are arbitrary:
    unsigned int width = 600;
    unsigned int height = 300;
    for (unsigned int screen_y = 0; screen_y < height; ++screen_y) {
        for (unsigned int screen_x = 0; screen_x < width; ++screen_x) {
            // Print centered at curr_pos:
            int x = screen_x + curr_pos.first - (width / 2);
            int y = -screen_y + curr_pos.second + (height / 2);
#else
    unsigned int width = lsm->yDim();
    unsigned int height = lsm->xDim();
    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            unsigned int screen_y = width - y;
            unsigned int screen_x = x;
#endif
            // Check occupancy at this position:
            if (lsm->gridOccupied(x, y))
                mvaddch(screen_y, screen_x, '#' | white);
//                    else if (lsm->gridPadded(x, y))
//                        mvaddch(screen_y, screen_x, '#' | green);
            else TB_ASSERT(! lsm->gridIllegal(Point(x, y)));

            // Print additional information, if there is any. This may override the
            // previous line, if there is more important information to print in a
            // given square.
            point_map::const_iterator itr = aux_map.find(Point(x, y));
            if (itr != aux_map.end()) {
                int_set s = itr->second;
                // value, action pair:
                typedef pair<int, int> vap;
                // Change the order to change the precedence with which the objects
                // are drawn:
                vap vaps[] = {
                    vap(CURR_POS, 'C' | blue | bold | blink),
                    vap(GOAL, 'G' | red | bold | blink),
                    vap(TRACE_RED, 'T' | red),
                    vap(TRACE_CYAN, 'T' | cyan),
                    vap(TRACE_PURPLE, 'T' | magenta),
                    //vap(OCCUPIED, 'O' | white),
                    vap(OCCUPIED_VIRTUALLY, 'O' | green),
                };
                vap value_action_pair;
                for (vap& value_action_pair : vaps) {
                    if (s.find(value_action_pair.first) != s.end()) {
                        mvaddch(screen_y, screen_x, value_action_pair.second);
                        break;
                    }
                }
            }
        }
    }
    refresh();
#endif
}

void TangentBug::printLocalSpaceMap() const {
    Logger::Level logLevel = opencog::Logger::FINE;
    if (logger().isEnabled(logLevel)) {
        std::string result = "\n";
        for (unsigned int x = 0; x < lsm->xDim(); x++) {
            for (unsigned int y = 0; y < lsm->yDim(); y++) {
                result += lsm->gridOccupied(x, y) ? 'O' :/*lsm->gridPadded(x,y)?'o':*/'-';
            }
            result += "\n";
        }
        logger().log(static_cast<opencog::Logger::Level>(logLevel),
                "\nLSM:%s\n", result.c_str());
    }
}

bool TangentBug::random_walk(int min_number_steps) {
    movement = RANDOM_WALK;
    int number_of_steps = std::max(random_walk_randomness, min_number_steps);

    logger().debug("TangentBug - Random walk of %d steps.",
                 number_of_steps);

    for (int i = 0; i < number_of_steps; ++i) {
        double angle = 2 * PI * rng.randDoubleOneExcluded();
        TBRay move_direction(cos(angle), sin(angle));

        // Free point in this direction:
        TBPoint pt = look_along_ray(curr_pos, move_direction, STEP_DISTANCE).last_point_before_hit;

        // At this point, do not check whether pt == curr_pos. Because what if
        // pt == curr_pos upon every iteration through the loop? That would mean no
        // movement would take place, random_walk() would get called repeatedly, and
        // the logic to return failure would never happen (because the action plan
        // would not get updated). This happens when the bug is completely trapped.

        //logger().fine("TangentBug::random_walk(): before gridIllegal");
        TB_ASSERT(! lsm->gridIllegal(pt));

        //_move(pt);
        if (! _move(pt)) return false;

        if (path_free_to_goal()) break;
    }

    // Next time take twice as many random steps.
    random_walk_randomness *= 2;
    return true;
}

void TangentBug::stop_ncurses() {
#ifdef TB_PRINT_NCURSES
    WINDOW * stdscr = initscr();
    initscr();
    start_color();
    cbreak();
    noecho();
    nonl();
    keypad(stdscr, true);
    wborder(stdscr, 0, 0, 0, 0, 0, 0, 0, 0);
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_BLUE, COLOR_BLACK);
    init_pair(5, COLOR_CYAN, COLOR_BLACK);
    init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(7, COLOR_WHITE, COLOR_BLACK);
    red = COLOR_PAIR(1);
    yellow = COLOR_PAIR(2);
    green = COLOR_PAIR(3);
    blue = COLOR_PAIR(4);
    cyan = COLOR_PAIR(5);
    magenta = COLOR_PAIR(6);
    white = COLOR_PAIR(7);
    bold = A_BOLD;
    blink = A_BLINK;
#endif
}

void TangentBug::init_ncurses() {
#ifdef TB_PRINT_NCURSES
    echo();
    nocbreak();
    nl();
    endwin();
#endif
}

