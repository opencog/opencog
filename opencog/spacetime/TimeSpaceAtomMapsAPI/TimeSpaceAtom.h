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

// TimeSpaceAtom.h
// make cicular buffer of struct of time+octomap array holding atom values
// api to put time space atom
// api to query time space for atom
// api to search and delete all atom occurences

#ifndef TimeSpaceAtom_H
#define TimeSpaceAtom_H
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

typedef std::chrono::system_clock::time_point time_pt;
typedef std::chrono::system_clock::duration duration_c;
typedef list<time_pt> time_list;

//data structures
struct TimeUnit
{
    time_pt t; duration_c duration;
    map<int, AtomOcTree> map_tree;
    TimeUnit(time_pt tp, duration_c d): t(tp), duration(d)
    {}
    bool operator==(time_pt tp)
    {
        return (tp >= t && tp <= t + duration);
    }
    //>,< not needed as only == search happens although created buffer should always be sorted, just simplifies a bit over search speed cost
    bool has_map(int handle)
    {
        auto it = map_tree.find(handle);
        return !(it == map_tree.end());
    }
};

class TimeSpaceAtom
{
public:
    //API
    unsigned int get_map_count();
    bool get_map_resolution(const int handle, double& res);
    bool get_current_time_range(time_pt& time_p, duration_c& duration);
    bool is_time_point_in_range(const time_pt& time_to_check, const time_pt& t,
                            const duration_c& duration)
    {
        return (time_to_check >= t && time_to_check < t + duration);
    }
    bool create_new_time_unit(const time_pt time_p, const duration_c duration);
    bool put_atom_at_current_time(const int map_handle, const point3d location,
                              const aHandle& ato);
    bool remove_atom_at_current_time(const int map_handle,
                                 const point3d location);
    bool remove_atom_at_time(time_pt tp, const int map_handle,
                          const point3d location);
    void remove_atom(const aHandle& ato);
    bool get_atom_current_time(const int map_handle, const point3d location,
                            aHandle& ato);
    bool get_atom_at_time(const time_pt& time_p, const int map_handle,
                       const point3d location, aHandle& ato);
    time_list get_times_of_atom_occurence_at_location(const int map_handle,
                                               const point3d location,
                                               const aHandle& ato);
    time_list get_times_of_atom_occurence_in_map(int map_handle, const aHandle& ato);
    point3d_list get_locations_of_atom_occurence_now(const int map_handle,
                                                const aHandle& ato);
    point3d_list get_locations_of_atom_occurence_at_time(const time_pt& time_p,
                                                   const int map_handle,
                                                   const aHandle& ato);
    //AtomList& GetAtomsInLocationBBXatTime();//BBX = bounding box

public:
    //constructor
    TimeSpaceAtom(unsigned int num_time_units, vector<double>map_res_meters);
private:
    unsigned int map_count;
    //each map may have translation rotation (orientation) co-ordinates managed by user
    map<int, double>map_res; //resolution of maps
    boost::circular_buffer<TimeUnit> time_circle;
    time_pt curr_time; duration_c curr_duration;
    bool created_once;
};
#endif
