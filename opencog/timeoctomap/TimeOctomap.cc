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

//TimeOctomap.cpp
#include "TimeOctomap.h"
//#include "octomap/OcTreeKey.h"
//#include <assert.h>
#include "opencog/util/oc_assert.h"

TimeOctomap::TimeOctomap(unsigned int num_time_units,
                         double map_res_meters,
                         duration_c time_resolution):
             map_res(map_res_meters),time_res(time_resolution),created_once(false),time_circle(num_time_units)
{
        
}

double
TimeOctomap::get_space_resolution()
{
    return map_res;
}

duration_c
TimeOctomap::get_time_resolution()
{
    return time_res;
}

bool
TimeOctomap::get_current_time_range(time_pt& time_p, duration_c& duration)
{
    if (time_circle.size() < 1) return false;
    time_p = curr_time;
    duration = time_res;
    return true;
}

bool
TimeOctomap::step_time_unit()
{
    time_pt time_p;
    duration_c duration=time_res;
    //ideally greater than check should be done
    if (created_once) {
        get_current_time_range(time_p,duration);//uniform time
        time_p+=duration;//uniform time duration
        /* do not delete this, may be useful in future
        if (is_time_point_in_range(time_p, curr_time, curr_duration) ||
                is_time_point_in_range(time_p + duration, curr_time, curr_duration)) {
            return false;
        }
        */
    }else{
         time_p=std::chrono::system_clock::now();
    }

    TimeUnit temp(time_p, duration);
    time_circle.push_back(temp);
    int i = time_circle.capacity() - 1;
    if (time_circle.size() < time_circle.capacity()) i = time_circle.size() - 1;
    /*
    for_each( map_res.begin(), map_res.end(), [&](pair<int, double> handle) {
        time_circle[i].map_tree[handle.first].setResolution(handle.second);
    }
    );
    */
    time_circle[i].map_tree.setResolution(map_res);

    curr_time = time_p;
    curr_duration = duration;
    created_once = true;
    return true;
}

bool
TimeOctomap::put_atom_at_current_time(const point3d location,
                                        const opencog::Handle& ato)
{
    //if (!created_once)return false;
    OC_ASSERT(created_once);
    int i = time_circle.capacity() - 1;
    if (time_circle.size() < time_circle.capacity()) i = time_circle.size() - 1;
    //if (!time_circle[i].has_map(handle)) return false;//may assert too
    time_circle[i].map_tree.updateNode(location, true);
    time_circle[i].map_tree.setNodeData(location, ato);
    return true;
}

bool
TimeOctomap::remove_atom_at_current_time_by_location(
                                       const point3d location)
{
    OC_ASSERT(created_once);
    int i = time_circle.capacity() - 1;
    if (time_circle.size() < time_circle.capacity()) i = time_circle.size() - 1;
    //time_circle[i].map_tree[map_handle].setNodeData(location,UndefinedHandle);
    time_circle[i].map_tree.updateNode(location, false);
    return true;
}

bool
TimeOctomap::remove_atom_at_time_by_location(time_pt tp,
                                const point3d location)
{
    OC_ASSERT(created_once);
    auto it = std::find(std::begin(time_circle), std::end(time_circle), tp); //time_circle.begin(),time_circle.end()
    if (it == std::end(time_circle))return false;
    //it->map_tree[map_handle].setNodeData(location,UndefinedHandle);
    it->map_tree.updateNode(location, false);
    return true;
}

bool
TimeOctomap::get_atom_current_time_at_location(
                                  const point3d location, opencog::Handle& ato)
{
    //
    OC_ASSERT(created_once);
    int i = time_circle.capacity() - 1;
    if (time_circle.size() < time_circle.capacity()) i = time_circle.size() - 1;
    OcTreeNode* result = time_circle[i].map_tree.search(location);
    if (result == nullptr) {
        ato = UndefinedHandle;
        return false;
    }
    ato = (static_cast<AtomOcTreeNode*>(result))->getData();
    if (ato == UndefinedHandle) return false;
    return true;
}

bool
TimeOctomap::get_atom_at_time_by_location(const time_pt& time_p,
                             const point3d location, opencog::Handle& ato)
{
    //
    OC_ASSERT(created_once);
    //find time in time circle time unit
    auto it = std::find(std::begin(time_circle), std::end(time_circle), time_p); //time_circle.begin(),time_circle.end()
    if (it == std::end(time_circle))return false;
    OcTreeNode* result = it->map_tree.search(location);
    if (result == nullptr) {
        ato = UndefinedHandle;
        return false;
    }
    ato = (static_cast<AtomOcTreeNode*>(result))->getData();
    if (ato == UndefinedHandle) return false;
    return true;
}//ok

time_list
TimeOctomap::get_times_of_atom_occurence_at_location(
                                                 const point3d location,
                                                 const opencog::Handle& ato)
{
    //
    time_list tl;
    for(auto tu = std::begin(time_circle), end = std::end(time_circle); (tu != end); tu++) {
        OcTreeNode* result = tu->map_tree.search(location);
        if (result == nullptr) {
            cout << "null ret by search" << endl;
            continue;
        }
        opencog::Handle ato_t = (static_cast<AtomOcTreeNode*>(result))->getData();
        if (ato_t != ato) {
            cout << "incorrect atom=" << ato_t;
            continue;
        }
        tl.push_back(tu->t);
    }
    return tl;
}//ok time_circle.begin is causing problem

time_list
TimeOctomap::get_times_of_atom_occurence_in_map(const opencog::Handle& ato)
{
    //
    time_list tl;
    for(auto tu = std::begin(time_circle), end = std::end(time_circle);
        (tu != end);
        tu++) {
        bool found = false;
        //go through all nodes and leafs of octomap to search atom
        for(AtomOcTree::tree_iterator it =
            tu->map_tree.begin_tree(),
            end = tu->map_tree.end_tree();
            it != end;
            ++it) {
            //
            if (it->getData() == ato) {
                found = true;
                break;
            }
        }
        if (found) tl.push_back(tu->t);
    }
    return tl;
}//ok
//FIXME: check time point within time duration and not just greater or less    
bool TimeOctomap::get_oldest_time_elapse_atom_observed(const opencog::Handle& ato,
                                            const time_pt& from_d,
                                            time_pt& result)
{
    time_list tl=get_times_of_atom_occurence_in_map(ato);
    //sort
    int sz=tl.size();
    if (sz<1) return false;
    tl.sort();
    if (from_d>tl.back()) return false;
    for (int i=0;i<sz;i++)
    {
       result=tl.front();
       tl.pop_front();
       if (result>=from_d)
       {
         return true;
       } 
    }
    return false;
}
    
bool TimeOctomap::get_last_time_elapse_atom_observed(const opencog::Handle& ato,
                                            const time_pt& till_d,
                                            time_pt& result)
{
    
    time_list tl=get_times_of_atom_occurence_in_map(ato);
    //sort
    int sz=tl.size();
    if (sz<1) return false;
    tl.sort();
    if (till_d<tl.front()) return false;
    for (int i=0;i<sz;i++)
    {
       result=tl.back();
       tl.pop_back();
       if (result<=till_d)
       {
         return true;
       } 
    }
    return false;
}

point3d_list
TimeOctomap::get_locations_of_atom_occurence_now(
                                              const opencog::Handle& ato)
{
    //
    OC_ASSERT(created_once);
    point3d_list pl;
    int i = time_circle.capacity() - 1;
    if (time_circle.size() < time_circle.capacity())
        i = time_circle.size() - 1;
    for(AtomOcTree::tree_iterator it =
        time_circle[i].map_tree.begin_tree(),
        end = time_circle[i].map_tree.end_tree();
        it != end;
        ++it) {
        //
        if (it->getData() == ato) pl.push_back(it.getCoordinate());
    }
    return pl;
}//ok

point3d_list
TimeOctomap::get_locations_of_atom_occurence_at_time(const time_pt& time_p,
                                                       const opencog::Handle& ato)
{
    //
    OC_ASSERT(created_once);
    point3d_list pl;
    auto it = std::find(std::begin(time_circle),
                        std::end(time_circle),
                        time_p); //time_circle.begin(),time_circle.end()
    if (it == std::end(time_circle))return point3d_list();
    for(AtomOcTree::tree_iterator ita =
        it->map_tree.begin_tree(),
        end = it->map_tree.end_tree();
        ita != end;
        ++ita) {
        //
        if (ita->getData() == ato) pl.push_back(ita.getCoordinate());
    }
    return pl;
}//ok

void
TimeOctomap::remove_atom_at_current_time(const opencog::Handle& ato)
{
    point3d_list pl;
    int i = time_circle.capacity() - 1;
    if (time_circle.size() < time_circle.capacity())
        i = time_circle.size() - 1;
    auto* tu=&time_circle[i];
            for(AtomOcTree::tree_iterator it2 =
                tu->map_tree.begin_tree(),
                endit2 = tu->map_tree.end_tree();
                it2 != endit2;
                ++it2) {
                if (it2->getData() == ato) {
                    pl.push_back(it2.getCoordinate());
                    it2->setData(UndefinedHandle);
                }
            }

            //cout<<"remove size="<<pl.size()<<endl;
            for(auto it3 = std::begin(pl), endit3 = std::end(pl);
                it3 != endit3;
                it3++) {
                tu->map_tree.deleteNode(*it3);
            }
}
void
TimeOctomap::remove_atom_at_time(const time_pt& time_p,const opencog::Handle& ato)
{
    point3d_list pl;
    auto tu = std::find(std::begin(time_circle),
                        std::end(time_circle),
                        time_p); //time_circle.begin(),time_circle.end()
    if (tu == std::end(time_circle))return;
            for(AtomOcTree::tree_iterator it2 =
                tu->map_tree.begin_tree(),
                endit2 = tu->map_tree.end_tree();
                it2 != endit2;
                ++it2) {
                if (it2->getData() == ato) {
                    pl.push_back(it2.getCoordinate());
                    it2->setData(UndefinedHandle);
                }
            }

            //cout<<"remove size="<<pl.size()<<endl;
            for(auto it3 = std::begin(pl), endit3 = std::end(pl);
                it3 != endit3;
                it3++) {
                tu->map_tree.deleteNode(*it3);
            }
}

void
TimeOctomap::remove_atom(const opencog::Handle& ato)
{
    //remove all occurences of atom in all maps at all times
    point3d_list pl;
    for(auto tu = std::begin(time_circle),
        end = std::end(time_circle);
        (tu != end);
        tu++) {
            pl.clear();
            for(AtomOcTree::tree_iterator it2 =
                tu->map_tree.begin_tree(),
                endit2 = tu->map_tree.end_tree();
                it2 != endit2;
                ++it2) {
                if (it2->getData() == ato) {
                    pl.push_back(it2.getCoordinate());
                    it2->setData(UndefinedHandle);
                }
            }

            //cout<<"remove size="<<pl.size()<<endl;
            for(auto it3 = std::begin(pl), endit3 = std::end(pl);
                it3 != endit3;
                it3++) {
                tu->map_tree.deleteNode(*it3);
            }

    }
}//hacked not perfect

//////spatial relations
bool 
TimeOctomap::get_a_location(const time_pt& time_p,const opencog::Handle& ato_target,point3d& location)
{
    //get atom location
    point3d_list target_list=get_locations_of_atom_occurence_at_time(time_p,ato_target);
    if (target_list.size()<1) return false;
    location=target_list.front();
    return true;
}

point3d
TimeOctomap::get_spatial_relations(const time_pt& time_p,const opencog::Handle& ato_obs,const opencog::Handle& ato_target,const opencog::Handle& ato_ref)
{
    //
    point3d res(-1.0,-1.0,-1.0);
    point3d v1,v2,v3;
    point3d d1,d2;
    if (!get_a_location(time_p,ato_obs,v1)) return res;
    if (!get_a_location(time_p,ato_target,v2)) return res;
    if (!get_a_location(time_p,ato_ref,v3)) return res;
    //calculate res
    return res;
}
    
bool //not normalized
TimeOctomap::get_direction_vector(const time_pt& time_p,const opencog::Handle& ato_obs,const opencog::Handle& ato_target,point3d& dir)
{
    //direction vector
    point3d tarh;
    point3d refh;
    if (!get_a_location(time_p,ato_target,tarh)) return false;
    if (!get_a_location(time_p,ato_obs,refh)) return false;
    dir= (tarh-refh);
    return true;
}
    
//2=far,1=near,0=touching, -1 unknown
int 
TimeOctomap::get_nearness(const time_pt& time_p,const opencog::Handle& ato_obs,const opencog::Handle& ato_target,const opencog::Handle& ato_ref)
{
    point3d dir1,dir2;
    if (!get_direction_vector(time_p,ato_obs,ato_target,dir1))return -1;
    if (!get_direction_vector(time_p,ato_obs,ato_ref,dir2))return -1;
    double ang=ang_vec(dir1,dir2);
    if (ang<=TOUCH_ANGLE) return 0;
    else if (ang<=NEAR_ANGLE) return 1;
    return 2;
}
    
//<-elipson=unknown,>=0 distance
double 
TimeOctomap::get_distance_between(const time_pt& time_p,const opencog::Handle& ato_target,const opencog::Handle& ato_ref)
{
    //get atom location
    point3d tarh;
    point3d refh;
    if (!get_a_location(time_p,ato_target,tarh)) return (-1.0);
    if (!get_a_location(time_p,ato_ref,refh)) return (-1.0);

    double dist=sqrt(sqr(tarh.x()-refh.x())+sqr(tarh.y()-refh.y())+sqr(tarh.z()-refh.z()));
    return dist;
}

