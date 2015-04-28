/*
 * opencog/spatial/TangentBug.h
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

#ifndef _SPATIAL__TANGENTBUG_H_
#define _SPATIAL__TANGENTBUG_H_

// The timeout is a hack that allows TangentBug to return false gracefully
// instead of walking forever (when the algorithm is not successful due to an
// implementation bug). Random walk is a hack that "shakes things up" when the
// bug is stuck. Turn off these hacks to do algorithm work:

// DO NOT #undef or #define TB_PRINT_NCURSES, but rather comment or uncomment
// the appropriate line in TangentBug/CMakeLists.txt.
#include <list>
#include <vector>
#include <boost/tuple/tuple.hpp>

#include <opencog/spatial/LocalSpaceMap2D.h>
#include <opencog/spatial/TangentBugCommons.h>

#ifdef TB_PRINT_NCURSES
#include <ncurses.h>
#endif

namespace opencog
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial
    {
        class TangentBug
        {

            bool debug_look_along_ray;
            bool debug_wall_follow;

        public:

            // typedefs - most of them are used to maintain old TangentBug
            // code consistence
            //typedef double Distance;
            //typedef Handle ObjectID;
            //typedef hashHandle ObjectIDHash;
            typedef boost::tuple < spatial::Point,
                spatial::TBPoint, int > PathPoint;
            typedef std::list<PathPoint> CalculatedPath;
            typedef spatial::LocalSpaceMap2D Map;
            //                typedef spatial::LocalSpaceMap2D<ObjectID,
            //                                                 Distance,
            //                                                 ObjectIDHash,
            //                                                 spatial::ObjMetaData> Map;
        protected:

            // Points on the map are considered contiguous if they are this close:
            static const int MAX_CONTIG_DISTANCE  = 5;
            static const int STEP_DISTANCE        = 10;
            static const int FOLLOW_STEP_DISTANCE = STEP_DISTANCE / 3;

            // There are circumstances where we can take bigger
            // steps than we normally allow: the main danger is coming
            // too close to obstacles because we didn't look before
            // we leapt (weren't close enough to pad the obstacle).
            static const int MAX_STEP_DISTANCE = STEP_DISTANCE * 3;

            // Approximate infinite sight: this approach enables me to not change any
            // methods, because all the old assumptions are still valid
            static const int SIGHT_DISTANCE = 1000000;

            // A bigger distance than should be relevant for any tangentbug calculation.
            // Useful for ensuring that unsigned int overflow has not occurred.
            // TODO: calculate this based on the height and width of the LocalSpaceMap
            static const unsigned int LARGE_NUM = 99999999;

            // Keep track of movement type -- useful for deciding what
            // size step to take:
            int movement;

            // LocalSpaceMap
            const Map* lsm;
            // Path computed
            CalculatedPath& calculatedPath;

            bool edge_following_recently_ended;
            point_map aux_map;
            TBPoint curr_pos;
            TBPoint prev_pos;
            TBPoint center;
            TBPoint goal;

            // Random number of random steps to take during a random walk:
            int random_walk_randomness;

            // TangentBug enumerations
            enum grid_status {
                GOAL,
                CURR_POS,
                OCCUPIED,
                OCCUPIED_VIRTUALLY,
                TRACE_PURPLE,
                TRACE_CYAN,
                TRACE_RED
            };

            enum direction {
                LEFT,
                RIGHT
            };

            enum movement_style {
                UNSET,
                NORMAL,
                TO_GOAL,
                FOLLOWING,
                RANDOM_WALK
            };

#ifndef TB_PRINT_NCURSES
            int red;
            int yellow;
            int green;
            int blue;
            int cyan;
            int magenta;
            int white;
            int bold;
            int blink;

            void getch() {
                ;
            } // This may be laying around.
#endif

            opencog::RandGen& rng;

        public:
            /**
             * Constructor
             *
             * Pass in an empty list that we can put points into.
             */
            TangentBug(const Map& lsm, CalculatedPath& calculatedPath);
            ~TangentBug();

            look_info look_along_ray(
                                     const TBPoint& initial_point,
                                     const TBRay& direction,
                                     double distance) const;

            void test_look_along_ray() const;

            void place_pet(spatial::Distance x, spatial::Distance y);
            void place_goal(spatial::Distance x, spatial::Distance y);

            void trace_path2(const TBPoint& pt1,
                             const TBPoint& pt2);

            void trace_path(const TBPoint &pt1,
                            const TBPoint &pt2,
                            int val = TRACE_CYAN);
            void untrace_path(const TBPoint &pt1,
                              const TBPoint &pt2,
                              int val = TRACE_CYAN);

            bool place_pet_randomly(const TBPoint& prob_center,
                                    int max_retries = 100);
            bool place_goal_randomly(const TBPoint& prob_center,
                                     bool allow_enclosed = false, int max_retries = 100);

            void print() const;
            void printLocalSpaceMap() const;

            bool random_walk(int min_number_steps = -1);

            const TBPoint& getCenter() const;
            const TBPoint& getCurrPos() const;
            const TBPoint& getGoal() const;
            bool seek_goal();

            void stop_ncurses();
            void init_ncurses();

        protected:
            void init_vars();

            TBRay dir_closest_wall();
            void cleanup_action_plan();

            bool path_free_to_goal();
            bool is_local_minimum(const TBPoint& pt) const;

            // Perform no checks, but go to a point:
            bool _move(const TBRay& r, double distance);
            bool _move(const TBPoint& dest);

            // There are a few scenarios that would cause the bug to move forever:
            //
            // -A strangely shaped map may cause a pet to start edge following, and "fall
            // off" its current edge, onto another. If d_reach less than the already
            // established (lower) d_followed, edge following will never break.
            //
            // -Random walk alternating with normal movement and/or edge following. It
            // can happen that the bug cannot see any points of discontinuity. This most
            // likely happens for one of two reasons:
            // 1) The bug is trapped in a very enclosed space.
            // 2) The bug is in a section of a map that it can't handle, and it keeps
            //    returning to a local minimum. It this case, it will most likely
            //    alternate between random walks, edge following, and normal movement,
            //    or just two of the three. This would be hard to detect with certainty.
            bool tb_timeout();

            bool follow_edge();
            double closest_distance_on_obstacle_to_goal(TBRay dir_to_object);

            bool reeval_subgoal(look_info &new_subgoal);

            std::vector<look_info> get_obstacle_endpoints();
            std::vector<look_info> get_visible_destinations2();
            std::vector<look_info> get_visible_destinations();

            void _place_pet(const TBPoint& pt);
            void _place_goal(const TBPoint& pt, bool allow_enclosed);

            inline bool closer_than_n_pixels(const look_info& look,
                                             int n, bool filter_away_from_goal) {
                if (filter_away_from_goal) {
                    return ((look.last_point_before_hit - curr_pos).length() < n &&
                            ((look.last_point_before_hit - goal).length() >
                             (curr_pos - goal).length()));
                } else {
                    return (curr_pos - look.last_point_before_hit).length() < n;
                }
            };

            // When you are next to a wall, this method returns the direction you should move:
            bool get_wall_tangent(int right_or_left, TBRay& tangent);
            bool path_free(const TBPoint& pt1,
                           const TBPoint& pt2) const;
            bool path_free(const TBPoint& pt1,
                           const TBRay& direction,
                           double distance) const;

        }; // class TangentBug

    } // namespace spatial
/** @}*/
} // opencog

#endif

