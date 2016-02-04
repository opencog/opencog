/*
 *
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 * All rights reserved.
 * License: New BSD
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

#ifndef TIME_SPACE_ATOM_H
#define TIME_SPACE_ATOM_H
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
typedef list<time_pt> TimeList;

//data structures
struct time_unit{
	time_pt t; duration_c duration;
	map<int,AtomOcTree> map_tree;
	time_unit(time_pt tp,duration_c d):t(tp),duration(d){}
	bool operator==(time_pt tp){
		return (tp>=t && tp<=t+duration);
	}
	//>,< not needed as only == search happens although created buffer should always be sorted, just simplifies a bit over search speed cost
	bool has_map(int handle){
		auto it=map_tree.find(handle);
		return !(it==map_tree.end());
	}
};

class TimeSpaceAtom{
	public:
	//API
	unsigned int GetMapCount();
	bool GetMapResolution(const int handle,double& res);
	bool GetCurrentTimeRange(time_pt& time_p,duration_c& duration);
	bool IsTimePointInRange(const time_pt& time_to_check,const time_pt& t,const duration_c& duration){
		return (time_to_check>=t && time_to_check<t+duration);
	}
	bool CreateNewTimeUnit(const time_pt time_p,const duration_c duration);
	bool PutAtomAtCurrentTime(const int map_handle,const point3d location,const aHandle& ato);
	bool RemoveAtomAtCurrentTime(const int map_handle,const point3d location);
	bool RemoveAtomAtTime(time_pt tp,const int map_handle,const point3d location);
	void RemoveAtom(const aHandle& ato);
	bool GetAtomCurrentTime(const int map_handle,const point3d location,aHandle& ato);
	bool GetAtomAtTime(const time_pt& time_p,const int map_handle,const point3d location,aHandle& ato);
	TimeList GetTimesOfAtomOccurenceAtLocation(const int map_handle,const point3d location,const aHandle& ato);
	TimeList GetTimesOfAtomOccurenceInMap(int map_handle,const aHandle& ato);
	point3d_list GetLocationsOfAtomOccurenceNow(const int map_handle,const aHandle& ato);
	point3d_list GetLocationsOfAtomOccurenceAtTime(const time_pt& time_p,const int map_handle,const aHandle& ato);
	//AtomList& GetAtomsInLocationBBXatTime();//BBX = bounding box
	
	public:
	//constructor
	TimeSpaceAtom(unsigned int num_time_units,vector<double>map_res_meters);
	private:
	unsigned int map_count;
	//each map may have translation rotation (orientation) co-ordinates managed by user
	map<int,double>map_res;//resolution of maps
	boost::circular_buffer<time_unit> time_circle;
	time_pt curr_time;duration_c curr_duration;
	bool created_once;
};
#endif
