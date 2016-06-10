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
        t=tu.t;duration=tu.duration;
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
    bool get_oldest_time_elapse_atom_observed(const opencog::Handle& ato,const time_pt& from_d,time_pt& result);
    //get the last atom observation before a time point
    bool get_last_time_elapse_atom_observed(const opencog::Handle& ato,
                                            const time_pt& till_d,
                                            time_pt& result);//throw
    //AtomList& GetAtomsInLocationBBXatTime();//BBX = bounding box

public:
    //constructor
    TimeOctomap(unsigned int num_time_units, double map_res_meters,
                duration_c time_resolution);
private:
    //each map may have translation rotation (orientation) co-ordinates managed by user
    double map_res; //resolution of maps
    duration_c time_res;
    boost::circular_buffer<TimeUnit> time_circle;
    time_pt curr_time; duration_c curr_duration;
    bool created_once;
};
#endif
