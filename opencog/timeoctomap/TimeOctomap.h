/*
 * TimeOctomap.h -- circular buffer of time+octomap array holding atoms
 *
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 * All rights reserved.
 * License: AGPL
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

using namespace std;
using namespace octomap;
using namespace opencog;

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

    // Return true if time-point is within this interval.
    bool operator==(time_pt tp)
    {
        return (tp >= t and tp <= t + duration);
    }

    TimeSlice& operator=(const TimeSlice& tu)
    {
        t = tu.t;
        duration = tu.duration;
        map_tree.clear();
        return *this;
    }

    // Remove the atom from this time-slice.
    void remove_atom(const Handle&);
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
    time_list get_times_of_atom_occurence_in_map(const Handle& ato);
    point3d_list get_locations_of_atom_occurence_now(const Handle&);
    point3d_list get_locations_of_atom_occurence_at_time(const time_pt&,
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

    bool get_oldest_time_locations_atom_observed(const Handle& ato,
                                                const time_pt& from_d,
                                                point3d_list& result);
    bool get_last_locations_of_atom_observed(const Handle& ato,
                                                          const time_pt& till_d,
                                                          point3d_list& result);
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
#endif
