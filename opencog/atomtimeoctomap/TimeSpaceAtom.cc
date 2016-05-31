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

//TimeSpaceAtom.cpp
#include "TimeSpaceAtom.h"
//#include "octomap/OcTreeKey.h"
//#include <assert.h>
#include "opencog/util/oc_assert.h"

TimeSpaceAtom::TimeSpaceAtom(unsigned int num_time_units,
                             double map_res_meters):
    time_circle(num_time_units),map_res(map_res_meters),created_once(false)
{
        
}

bool
TimeSpaceAtom::get_map_resolution(double& res)
{
    res = map_res;
    return true;
}

bool
TimeSpaceAtom::get_current_time_range(time_pt& time_p, duration_c& duration)
{
    if (time_circle.size() < 1) return false;
    time_p = curr_time;
    duration = curr_duration;
    return true;
}

bool
TimeSpaceAtom::create_new_time_unit(const time_pt time_p,
                                 const duration_c duration)
{
    //ideally greater than check should be done
    if (created_once) {
        if (is_time_point_in_range(time_p, curr_time, curr_duration) ||
                is_time_point_in_range(time_p + duration, curr_time, curr_duration)) {
            return false;
        }
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
TimeSpaceAtom::put_atom_at_current_time(const point3d location,
                                        const aHandle& ato)
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
TimeSpaceAtom::remove_atom_at_current_time_by_location(
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
TimeSpaceAtom::remove_atom_at_time_by_location(time_pt tp,
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
TimeSpaceAtom::get_atom_current_time_at_location(
                                  const point3d location, aHandle& ato)
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
TimeSpaceAtom::get_atom_at_time_by_location(const time_pt& time_p,
                             const point3d location, aHandle& ato)
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
TimeSpaceAtom::get_times_of_atom_occurence_at_location(
                                                 const point3d location,
                                                 const aHandle& ato)
{
    //
    time_list tl;
    for(auto tu = std::begin(time_circle), end = std::end(time_circle); (tu != end); tu++) {
        OcTreeNode* result = tu->map_tree.search(location);
        if (result == nullptr) {
            cout << "null ret by search" << endl;
            continue;
        }
        aHandle ato_t = (static_cast<AtomOcTreeNode*>(result))->getData();
        if (ato_t != ato) {
            cout << "incorrect atom=" << ato_t;
            continue;
        }
        tl.push_back(tu->t);
    }
    return tl;
}//ok time_circle.begin is causing problem

time_list
TimeSpaceAtom::get_times_of_atom_occurence_in_map(const aHandle& ato)
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
    };
    return tl;
}//ok

point3d_list
TimeSpaceAtom::get_locations_of_atom_occurence_now(
                                              const aHandle& ato)
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
TimeSpaceAtom::get_locations_of_atom_occurence_at_time(const time_pt& time_p,
                                                       const aHandle& ato)
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
TimeSpaceAtom::remove_atom_at_current_time(const aHandle& ato)
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
TimeSpaceAtom::remove_atom_at_time(const time_pt& time_p,const aHandle& ato)
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
TimeSpaceAtom::remove_atom(const aHandle& ato)
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
