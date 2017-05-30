/*
 * TimeOctomap.h -- circular buffer of time+octomap array holding atoms
 *
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 *
 * All rights reserved.
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

#ifndef TimeOctomap_H
#define TimeOctomap_H

#include <math.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <string>

#include <boost/circular_buffer.hpp>
#include "AtomOcTree.h"

namespace opencog
{
using namespace octomap;

typedef std::chrono::system_clock::time_point time_pt;
typedef std::chrono::system_clock::duration duration_c;
typedef std::list<time_pt> time_list;

#define DEG2RAD(deg) (M_PI/180.0)*deg
#define TOUCH_ANGLE DEG2RAD(10.0)
#define NEAR_ANGLE DEG2RAD(20.0)

struct TimeSlice
{
    time_pt t;
    duration_c duration;
    AtomOcTree map_tree;
    TimeSlice(time_pt tp, duration_c d): t(tp), duration(d) {}

    /// Return true if time-point is within this interval.
    /// The end-point is NOT considfrered to be a part of the interval,
    /// and this is important: otherwise, a time-point could be in two
    /// adjacent intervals, and this will mess up search queries.
    bool operator==(time_pt tp)
    {
        return (t <= tp and tp < t + duration);
    }

    TimeSlice& operator=(const TimeSlice& tu)
    {
        t = tu.t;
        duration = tu.duration;
        map_tree.clear();
        return *this;
    }

    // Store an atom at `location`, for this timeslice
    void insert_atom(const point3d&, const Handle&);

    // Remove the atom from this time-slice.
    void remove_atom(const Handle&);
    void remove_atoms_at_location(const point3d&);

    // Get the atom at location
    Handle get_atom_at_location(const point3d&);

    // Get the locations of an atom.
    point3d_list get_locations(const Handle&);
};

class TimeOctomap
{
public:
    // Return the spatial resolutionof the map, in meters
    double get_space_resolution();
    // Return the time-resolution of the map (in what units???)
    duration_c get_time_resolution();

    // Get ... ??? something.
    int get_time_units() { return time_circle.capacity(); }

    // Get the start-time time point and length of the current time-slice
    time_pt get_current_time() { return curr_time; }

    //helper function to check if a time point is within the Time unit time range
    // Create a new time-slice, and make it the current time-slice,
    // closing off the previous one. It will come immediately after the
    // previous slice, and will not overlap with it.
    void step_time_unit();

    // Return a pointer to the time-slice containing the point in time.
    // Return nullptr if the time-point is not within the range of this
    // map.
    TimeSlice *find(const time_pt& time_p);

    TimeSlice& get_current_timeslice();

    bool is_auto_step_time_on();
    void auto_step_time(bool astep);

    // Store an atom at `location`, for the current timeslice
    void insert_atom(const point3d&, const Handle&);

    void remove_atoms_at_location(const point3d&);
    void remove_atom_at_time_by_location(time_pt, const point3d&);

    // Remove the atom from the current timeslice
    void remove_atom_at_current_time(const Handle&);
    void remove_atom_at_time(const time_pt&, const Handle&);

    // Remove all occurences of atom in all time-slices
    void remove_atom(const Handle&);

    // Get atom at the given location in the current time-slice.
    Handle get_atom_at_location(const point3d&);

    Handle get_atom_at_time_by_location(const time_pt&, const point3d&);
    time_list get_times_of_atom_occurence_at_location(
                                               const point3d&,
                                               const Handle& ato);
    time_list get_timeline(const Handle&);
    point3d_list get_locations_of_atom(const Handle&);
    point3d_list get_locations_of_atom_at_time(const time_pt&,
                                                         const Handle&);
    //get the first atom observation after a time point
    bool get_oldest_time_elapse_atom_observed(const Handle& ato,const time_pt& from_d,time_pt& result);//?return location too?
    //get the last atom observation before a time point
    bool get_last_time_elapse_atom_observed(const Handle& ato,
                                            const time_pt& till_d,
                                            time_pt& result);
    bool get_last_time_before_elapse_atom_observed(const Handle& ato,
                                                  const time_pt& till_d,
                                                  time_pt& result);

    point3d_list get_oldest_locations(const Handle&, const time_pt&);
    point3d_list get_newest_locations(const Handle&, const time_pt&);

    //AtomList& GetAtomsInLocationBBXatTime();//BBX = bounding box
    //insert point cloud
    //find ray intersection

    //////////spatial-relations-queries
    //should be true-false-unknown
    //assuming z orientation is fixed i.e. sky relative to ground
    //assuming observer is looking towards reference
    //target is $x of reference
    //y=2-right,1-left,0-aligned (>elipson,<-elipson)
    //z=2-above,1-below,0-aligned
    //x=2-ahead,1-behind,0 - aligned
    point3d get_spatial_relations(const time_pt& time_p, const Handle& ato_obs,const Handle& ato_target,const Handle& ato_ref);
    //not normalized: direction vector -> (target-observer)
    bool get_direction_vector(const time_pt& time_p,const Handle& ato_obs,const Handle& ato_target,point3d&);
    //got to another nearness for physical distance, this one is angular
    //2=far,1=near,0=touching, -1 unknown
    int get_angular_nearness(const time_pt&, const Handle& ato_obs, const Handle& ato_target, const Handle& ato_ref);
    //<-elipson=unknown,>=0 distance
    double get_distance_between(const time_pt&, const Handle&, const Handle&);
    bool get_a_location(const time_pt&, const Handle&, point3d&);
public:
    //constructor
    TimeOctomap(unsigned int num_time_units, double map_res_meters,
                duration_c time_resolution);
    ~TimeOctomap();
    inline double sqr(double a){return (a*a);}
    inline double dot(point3d a,point3d b){return (a.x()*b.x()+a.y()*b.y()+a.z()*b.z());}
    inline double mag(point3d a){return sqrt(sqr(a.x())+sqr(a.y())+sqr(a.z()));}
    inline double ang_vec(point3d a,point3d b)
    {
        //FIXME: Test this hueristic to be correct
        double num=dot(a,b);
        double den=mag(a)*mag(b);
        double diff=abs(mag(a)-mag(b));
        if (den<1e-9) // num might be greater or equal to space_res
        {
            if (diff<1e-3)
                return 0.0; // magic number
            else
                return M_PI;
        }
        return acos(num/den);
    }
    inline void rot2d(double x,double y,double th,double &rx,double &ry)
    {
        rx=x*cos(th)-y*sin(th);
        ry=x*sin(th)+y*cos(th);
    }
private:
    // Each different map may have translation and rotation (orientation)
    // co-ordinates managed by user
    double map_res; // spetial resolution of the map
    duration_c time_res;
    boost::circular_buffer<TimeSlice> time_circle;
    time_pt curr_time;
    void auto_timer();
    bool auto_step;
    std::mutex mtx, mtx_auto;
    std::thread g_thread;
};
}

namespace std
{
ostream& operator<<(ostream&, const opencog::time_pt&);
ostream& operator<<(ostream&, const opencog::duration_c&);
ostream& operator<<(ostream&, const opencog::time_list&);
ostream& operator<<(ostream&, const octomap::point3d_list&);
}

#endif
