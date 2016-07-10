/*
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

// TimeOctomap.h
// make cicular buffer of struct of time+octomap array holding atom values
// api to put time space atom
// api to query time space for atom
// api to search and delete all atom occurences

#ifndef TimeOctomap_H
#define TimeOctomap_H
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <list>
#include <map>
#include <string>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "AtomOcTree.h"
//#include "AtomOcTreeNode.h"
using namespace std;
using namespace octomap;
//high_resolution_clock
//typedef std::chrono::high_resolution_clock::time_point time_pt;
//typedef std::chrono::high_resolution_clock::duration duration_c;

typedef std::chrono::system_clock::time_point time_pt;
typedef std::chrono::system_clock::duration duration_c;
typedef list<time_pt> time_list;
//constants below require tweaking or better logic
#define PI 3.142
#define DEG2RAD(deg) (PI/180.0)*deg
#define TOUCH_ANGLE DEG2RAD(10.0)
#define NEAR_ANGLE DEG2RAD(20.0)

//data structures
struct TimeUnit
{
    time_pt t; duration_c duration;
    AtomOcTree map_tree;
    TimeUnit(time_pt tp, duration_c d): t(tp), duration(d)
    {}
    bool operator==(time_pt tp)
    {
        return (tp >= t && tp <= t + duration);
    }

    TimeUnit& operator=(const TimeUnit& tu)
    {
        t=tu.t; duration=tu.duration;
        map_tree.clear();
        return *this;
    }

    //>,< not needed as only == search happens although created buffer should always be sorted, just simplifies a bit over search speed cost
};

class TimeOctomap
{
public:
    //API
    double get_space_resolution();//map resolution in meters
    duration_c get_time_resolution();
    int get_time_units(){
      return time_circle.capacity();
    }
    //current time unit time point and time duration are queried
    bool get_current_time_range(time_pt& time_p, duration_c& duration);
    //helper function to check if a time point is within the Time unit time range
    bool is_time_point_in_range(const time_pt& time_to_check, const time_pt& t, const duration_c& duration)
    {
        return (time_to_check >= t && time_to_check < t + duration);
    }
    //make a new time unit for storage,
    //should not overlap a previous time unit
    //and should fall after the previous time unit
    bool step_time_unit();//step_time_unit
    bool is_auto_step_time_on();
    void auto_step_time(bool astep);
    //store an atom at coordinates in map
    bool put_atom_at_current_time(const point3d location,
                              const opencog::Handle& ato);
    bool remove_atom_at_current_time_by_location(const point3d location);
    bool remove_atom_at_time_by_location(time_pt tp,const point3d location);
    void remove_atom_at_current_time(const opencog::Handle& ato);
    void remove_atom_at_time(const time_pt& time_p,const opencog::Handle& ato);
    void remove_atom(const opencog::Handle& ato);
    //get atom at current time unit
    bool get_atom_current_time_at_location(const point3d location,
                            opencog::Handle& ato);
    bool get_atom_at_time_by_location(const time_pt& time_p,
                       const point3d location, opencog::Handle& ato);
    time_list get_times_of_atom_occurence_at_location(
                                               const point3d location,
                                               const opencog::Handle& ato);
    time_list get_times_of_atom_occurence_in_map(const opencog::Handle& ato);
    point3d_list get_locations_of_atom_occurence_now(const opencog::Handle& ato);
    point3d_list get_locations_of_atom_occurence_at_time(const time_pt& time_p,const opencog::Handle& ato);
    //get the first atom observation after a time point
    bool get_oldest_time_elapse_atom_observed(const opencog::Handle& ato,const time_pt& from_d,time_pt& result);//?return location too?
    //get the last atom observation before a time point
    bool get_last_time_elapse_atom_observed(const opencog::Handle& ato,
                                            const time_pt& till_d,
                                            time_pt& result);//throw
    //AtomList& GetAtomsInLocationBBXatTime();//BBX = bounding box
    //insert point cloud
    //find ray intersection

    //////////spatial-relations-queries
    //should be true-false-unknown
    //assuming z orientation is fixed i.e. sky relative to ground
    //assuming observer is looking towards reference
    //target is $x of reference
    //y=2-right,1-left,0-aligned,-1-unknown (>elipson,<-elipson)
    //z=2-above,1-below,0-aligned, -1 unknown
    //x=2-ahead,1-behind,0 - aligned, -1 unknown
    point3d get_spatial_relations(const time_pt& time_p,const opencog::Handle& ato_obs,const opencog::Handle& ato_target,const opencog::Handle& ato_ref);
    //not normalized: direction vector -> (target-observer)
    bool get_direction_vector(const time_pt& time_p,const opencog::Handle& ato_obs,const opencog::Handle& ato_target,point3d&);
    //got to another nearness for physical distance, this one is angular
    //2=far,1=near,0=touching, -1 unknown
    int get_angular_nearness(const time_pt& time_p,const opencog::Handle& ato_obs,const opencog::Handle& ato_target,const opencog::Handle& ato_ref);
    //<-elipson=unknown,>=0 distance
    double get_distance_between(const time_pt& time_p,const opencog::Handle& ato_target,const opencog::Handle& ato_ref);
    bool get_a_location(const time_pt& time_p,const opencog::Handle& ato_target,point3d&);
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
        if (den<1e-9)//num might be greater or equal to space_res
        {
            if (diff<1e-3)
                return 0;//magic number
            else
                return PI;//Pi radians
        }
        return acos(num/den);
    }
    inline void rot2d(double x,double y,double th,double &rx,double &ry)
    {
        rx=x*cos(th)-y*sin(th);
        ry=x*sin(th)+y*cos(th);
    }
private:
    //each map may have translation rotation (orientation) co-ordinates managed by user
    double map_res; //resolution of maps
    duration_c time_res;
    boost::circular_buffer<TimeUnit> time_circle;
    time_pt curr_time; duration_c curr_duration;
    bool created_once;
    void auto_timer();
    bool auto_step;
    std::mutex mtx,mtx_auto;
    std::thread g_thread;
};
#endif
