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
#include <thread>
#include <mutex>

#include <boost/circular_buffer.hpp>
#include "AtomOcTree.h"

namespace opencog
{
using namespace octomap;

typedef std::chrono::system_clock::time_point time_pt;
typedef std::chrono::system_clock::duration duration_c;
typedef std::list<time_pt> time_list;

#define DEG2RAD(deg) ( (M_PI/180.0)*deg )
#define TOUCH_ANGLE DEG2RAD(10.0)
#define NEAR_ANGLE DEG2RAD(20.0)

template <typename T>
struct TimeSlice
{
    time_pt t;
    duration_c duration;
    AtomOcTree<T> map_tree;
    std::vector<T> temporal;
    TimeSlice(time_pt tp, duration_c d): t(tp), duration(d) {}

    /// Return true if time-point is within this interval.
    /// The end-point is NOT considered to be a part of the interval,
    /// and this is important: otherwise, a time-point could be in two
    /// adjacent intervals, and this will mess up search queries.
    bool operator==(time_pt tp)
    {
        return (t <= tp and tp < t + duration);
    }

    TimeSlice& operator=(const TimeSlice<T>& tu)
    {
        t = tu.t;
        duration = tu.duration;
        map_tree.clear();
        return *this;
    }

    // Store an atom at `location`, for this timeslice
    void insert_atom(const point3d& location, const T& ato)
    {
        map_tree.updateNode(location, true);
        map_tree.setNodeData(location, ato);
    }

    void insert_atom(const T& ato)
    {
        temporal.push_back(ato);
    }

    // Remove the atom from this time-slice.
    void remove_atom(const T& ato)
    {
        for (auto it = temporal.begin(); it != temporal.end(); ++it) {
            if (*it == ato) {
                temporal.erase(it);
            }
        }

        point3d_list pl;
        for (typename AtomOcTree<T>::tree_iterator it2 = map_tree.begin_tree(),
                endit2 = map_tree.end_tree();
                it2 != endit2;
                ++it2)
        {
            if (it2->getData() == ato) {
                pl.push_back(it2.getCoordinate());
                it2->setData(T()); //FIXME this requires default constructor always??
            }
        }

        for (auto& p : pl)
            map_tree.deleteNode(p);
    }

    void remove_atoms_at_location(const point3d& location)
    {
        map_tree.updateNode(location, false);
    }

    // Get the atom at location
    T get_atom_at_location(const point3d& location)
    {
        OcTreeNode* result = map_tree.search(location);
        if (result == nullptr or not map_tree.isNodeOccupied(result)) return T(); //FIXME
        return (static_cast<AtomOcTreeNode<T>*>(result))->getData();
    }

    /// get_locations -- get zero, one or more locations (3D coordinates)
    /// of an atom in this time-slice.  A time-slice does allow a single
    /// atom to be present at multiple locations, and this will return all
    /// of them.  This returns an empty list if the atom does not appear
    /// in the timeslice.
    // Get the locations of an atom.
    point3d_list get_locations(const T& ato)
    {
        point3d_list pl;
        for (typename AtomOcTree<T>::tree_iterator ita = map_tree.begin_tree(),
                end = map_tree.end_tree(); ita != end; ++ita) {
            if (ita->getData() == ato)
                pl.push_back(ita.getCoordinate());
        }
        return pl;
    }
};


template <typename T>
class TimeOctomap
{
public:
    // Return the spatial resolutionof the map, in meters
    double get_space_resolution()
    {
        return map_res;
    }
    // Return the time-resolution of the map (in what units???)
    duration_c get_time_resolution()
    {
        return time_res;
    }

    // Get ... ??? something.
    int get_time_units() { return time_circle.capacity(); }

    // Get the start-time time point and length of the current time-slice
    time_pt get_current_time() { return curr_time; }

    //helper function to check if a time point is within the Time unit time range
    // Create a new time-slice, and make it the current time-slice,
    // closing off the previous one. It will come immediately after the
    // previous slice, and will not overlap with it.
    void step_time_unit()
    {
        std::lock_guard<std::mutex> lgm(mtx);
        curr_time += time_res;
        TimeSlice<T> tu(curr_time, time_res);
        tu.map_tree.setResolution(map_res);
        time_circle.push_back(tu);
    }

    // Return a pointer to the time-slice containing the point in time.
    // Return nullptr if the time-point is not within the range of this
    // map.
    TimeSlice<T> *find(const time_pt& time_p)
    {
        for (TimeSlice<T>& tu : time_circle)
            if (tu == time_p) return &tu;
        return nullptr;
    }

    TimeSlice<T>& get_current_timeslice()
    {
        // XXX FIXME - can't we just use size() always ???
        int i = time_circle.capacity() - 1;
        if (time_circle.size() < time_circle.capacity())
            i = time_circle.size() - 1;
        return time_circle[i];
    }

    bool is_auto_step_time_on()
    {
        return auto_step;
    }

    void auto_step_time(bool astep)
    {
        std::lock_guard<std::mutex> t_mtx(mtx_auto);
        if (auto_step == astep) return;
        auto_step = astep;
        if (astep) auto_timer();
        else g_thread.join();
    }

    // Store an atom at `location`, for the current timeslice
    void insert_atom(const point3d& location, const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        TimeSlice<T>& tu = get_current_timeslice();
        tu.insert_atom(location, ato);
    }

    void insert_atom(const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        TimeSlice<T>& tu = get_current_timeslice();
        tu.insert_atom(ato);
    }

    void remove_atoms_at_location(const point3d& location)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        TimeSlice<T>& tu = get_current_timeslice();
        tu.remove_atoms_at_location(location);
    }

    void remove_atom_at_time_by_location(time_pt tp,
                                 const point3d& location)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        auto tu = find(tp);
        if (tu == nullptr) return;
        tu->remove_atoms_at_location(location);
    }

    // Remove the atom from the current timeslice
    void remove_atom_at_current_time(const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        TimeSlice<T>& tu = get_current_timeslice();
        tu.remove_atom(ato);
    }

    void remove_atom_at_time(const time_pt& time_p, const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        auto tu = find(time_p);
        if (tu == nullptr) return;
        tu->remove_atom(ato);
    }

    // Remove all occurences of atom in all time-slices
    void remove_atom(const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        for (auto& tu : time_circle) tu.remove_atom(ato);
    }

    // Get atom at the given location in the current time-slice.
    T get_atom_at_location(const point3d& location)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        TimeSlice<T>& tu = get_current_timeslice();
        return tu.get_atom_at_location(location);
    }

    T get_atom_at_time_by_location(const time_pt& time_p,
                                   const point3d& location)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        auto tu = find(time_p);
        if (tu == nullptr) return T(); //FIXME sholdn't need a default constructor. return sth else.
        return tu->get_atom_at_location(location);
    }

    time_list get_times_of_atom_occurence_at_location(const point3d& location,
                                                      const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        time_list tl;
        for (auto& tu : time_circle)
        {
            T ato_t = tu.get_atom_at_location(location);
            if (ato_t != ato) continue;

            tl.push_back(tu.t);
        }
        return tl;
    }

    /// get_timeline - Get the sequence of points in time at which the
    /// atom appears in the map.  There will be one time-point for each
    /// time-slice in which the atom appears.
    time_list get_timeline(const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        time_list tl;
        for (auto& tu : time_circle) {
            // Go through all nodes of the octomap, searching for the atom
            // FIXME -- the octomap should provide this as a method.
            // We should not have to search for this ourselves.
            for (auto& nod : tu.map_tree) {
                if (nod.getData() == ato) {
                    tl.push_back(tu.t);
                    break;
                }
            }

            for (auto& data : tu.temporal) {
                if (data == ato) {
                    tl.push_back(tu.t);
                    break;
                }
            }
        }
        return tl;
    }

    point3d_list get_locations_of_atom(const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        TimeSlice<T>& tu = get_current_timeslice();
        return tu.get_locations(ato);
    }

    /// get_locations_of_atom_at_time -- get zero, one or more locations
    /// (3D coordinates) of an atom at the given time.  The map does allow
    /// a single atom to be present at multiple locations, and this will
    /// retreive all of them.  If the atom is not present at this time,
    /// an empty list will be reserved.
    point3d_list get_locations_of_atom_at_time(const time_pt& time_p,
                                               const T& ato)
    {
        std::lock_guard<std::mutex> lgm(mtx);
        TimeSlice<T> * it = find(time_p);
        if (it == nullptr) return point3d_list();
        return it->get_locations(ato);
    }

    //get the first atom observation after a time point
    //FIXME: check time point within time duration and not just greater or less
    bool get_oldest_time_elapse_atom_observed(const T& ato,
                                              const time_pt& from_d,
                                              time_pt& result)
    { //?return location too?
        time_list tl = get_timeline(ato);
        // XXX FIXME when/why would this ever NOT be sorted?
        tl.sort();
        for (auto& tp : tl) {
            if (tp >= from_d) {
                result = tp;
                return true;
            }
        }
        return false;
    }

    /// get_last_time_elapse_atom_observed -- get the latest time that
    /// the atom was obsserved, as long as it was observed more recently
    /// than `from_d`.  XXX FIXME -- this is a kind-of pointless
    /// operation to do, as the user is quite capable of doing the math,
    /// themselves, to perform the subtraction.  So this method, and the
    /// others like it, should be removed from the API.  This will simplify
    /// the API a lot. todo -- fixme later.

    //get the last atom observation before a time point
    bool get_last_time_elapse_atom_observed(const T& ato,
                                            const time_pt& from_d,
                                            time_pt& result)
    {
        time_list tl = get_timeline(ato);
        if (0 == tl.size()) return false;

        // XXX FIXME -- when would this ever NOT be sorted??
        tl.sort();
        if (from_d > tl.back() and tl.back() != from_d)
        {
            return false;
        }
        result = tl.back();
        return true;
    }

    bool get_last_time_before_elapse_atom_observed(const T& ato,
            const time_pt& till_d,
            time_pt& result)
    {
        time_list tl = get_timeline(ato);
        if (0 == tl.size()) return false;
        tl.sort();

        if (till_d < tl.front()) return false;

        for (auto& tp : tl) {
            if (tp <= till_d) {
                result = tp;
                return true;
            }
        }
        return false;
    }

    point3d_list get_oldest_locations(const T& ato, const time_pt& from_d)
    {
        time_pt tpt;
        if (not get_oldest_time_elapse_atom_observed(ato, from_d, tpt))
            return point3d_list();
        return get_locations_of_atom_at_time(tpt, ato);
    }

    point3d_list get_newest_locations(const T& ato, const time_pt& till_d)
    {
        time_pt tpt;
        if (not get_last_time_elapse_atom_observed(ato, till_d, tpt))
            return point3d_list();
        return get_locations_of_atom_at_time(tpt, ato);
    }

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
    point3d get_spatial_relations(const time_pt& time_p, const T& ato_obs,
                                  const T& ato_target, const T& ato_ref)
    {
        // not normalized: direction vector -> (target-observer)
        // reference and observer cant be the same location
        point3d res(-1.0, -1.0, -1.0);
        point3d v1, v2, v3;
        double eps = map_res*0.1;
        if (!get_a_location(time_p, ato_obs, v1))
            return res;
        if (!get_a_location(time_p, ato_target, v2))
            return res;
        if (!get_a_location(time_p, ato_ref, v3))
            return res;
        //calculate res
        //translate obs to origin and relatively move others
        //rotate vector obs target to be on an axis, relatively rotate ref
        //see if on left or right, up or down, front or back
        point3d orv = v3 - v1;
        if (abs(orv.x()) <= eps && abs(orv.y()) <= eps && abs(orv.z()) <= eps)
            return res;
        point3d otv = v2 - v1;
        double th = atan2(orv.y(), orv.x());
        double cx, cy, dx, dy;
        //rotate around z to zx plane
        rot2d(orv.x(), orv.y(), -1.0*th, cx, cy);
        orv = point3d(cx, 0.0, orv.z());
        //rotate around z
        rot2d(otv.x(), otv.y(), -1.0*th, dx, dy);
        otv = point3d(dx, dy, otv.z());
        th = atan2(orv.z(), orv.x());
        //rotate around y to x axis
        rot2d(orv.x(), orv.z(), -1.0*th, cx, cy);
        orv = point3d(cx, 0.0, 0.0);
        //rotate around y axis
        rot2d(otv.x(), otv.z(), -1.0*th, dx, dy);
        otv = point3d(dx, otv.y(), dy);
        res = otv - orv;

        //x .. ahead=2, behind=1, aligned=0
        //y .. right, left, align
        //z .. above, below, align
        double px, py, pz;
        if (res.x() > eps)
            px = 1.0;
        else if (res.x() < -1.0*eps)
            px = 2.0;
        else
            px = 0.0;

        if (res.y() > eps)
            py = 2.0;
        else if (res.y() < -1.0*eps)
            py = 1.0;
        else
            py = 0.0;

        if (res.z() > eps)
            pz = 2.0;
        else if (res.z() < -1.0*eps)
            pz = 1.0;
        else
            pz = 0.0;
        res = point3d(px, py, pz);
        return res;
    }

    bool get_direction_vector(const time_pt& time_p, const T& ato_obs,
                               const T& ato_target, point3d& dir)
    {
        // direction vector
        point3d tarh;
        point3d refh;
        if (!get_a_location(time_p, ato_target, tarh))
            return false;
        if (!get_a_location(time_p, ato_obs, refh))
            return false;
        dir = tarh - refh;
        return true;
    }

    //got to another nearness for physical distance, this one is angular
    //2=far,1=near,0=touching, -1 unknown
    int get_angular_nearness(const time_pt& time_p, const T& ato_obs,
                             const T& ato_target, const T& ato_ref)
    {
        point3d dir1, dir2;
        if (not get_direction_vector(time_p, ato_obs, ato_target, dir1))
            return -1;
        if (not get_direction_vector(time_p, ato_obs, ato_ref, dir2))
            return -1;
        double ang = ang_vec(dir1, dir2);
        if (ang <= TOUCH_ANGLE)
            return 0;
        else if (ang <= NEAR_ANGLE)
            return 1;
        return 2;
    }

    //<-elipson=unknown,>=0 distance
    double get_distance_between(const time_pt& time_p, const T& ato_target,
                                const T& ato_ref)
    {
        //get atom location
        point3d tarh;
        point3d refh;
        if (!get_a_location(time_p, ato_target, tarh))
            return (-1.0);
        if (!get_a_location(time_p, ato_ref, refh))
            return (-1.0);

        double dist = sqrt(sqr(tarh.x()-refh.x())+sqr(tarh.y()-refh.y())+sqr(tarh.z()-refh.z()));
        return dist;
    }

    //////spatial relations
    //later instead of get a location, use get nearest location or get furthest location
    bool get_a_location(const time_pt& time_p, const T& ato_target, point3d& location)
    {
        //get atom location
        point3d_list target_list = get_locations_of_atom_at_time(time_p, ato_target);
        if (target_list.size() < 1)
            return false;
        location = target_list.front();
        return true;
    }

public:
    //constructor
    TimeOctomap(unsigned int num_time_units,
                double map_res_meters,
                duration_c time_resolution) :
                map_res(map_res_meters),
                time_res(time_resolution),
                time_circle(num_time_units),
                auto_step(false)
    {
        curr_time = std::chrono::system_clock::now();
        TimeSlice<T> tu(curr_time, time_res);
        tu.map_tree.setResolution(map_res);
        time_circle.push_back(tu);
    }

    ~TimeOctomap()
    {
        auto_step_time(false);
    }

    inline double sqr(double a) { return (a*a); }
    inline double dot(point3d a, point3d b) {
        return (a.x()*b.x()+a.y()*b.y()+a.z()*b.z());
    }
    inline double mag(point3d a) {
        return sqrt(sqr(a.x())+sqr(a.y())+sqr(a.z()));
    }
    inline double ang_vec(point3d a, point3d b)
    {
        //FIXME: Test this hueristic to be correct
        double num = dot(a, b);
        double den = mag(a)*mag(b);
        double diff = abs(mag(a) - mag(b));
        if (den < 1e-9) // num might be greater or equal to space_res
        {
            if (diff < 1e-3)
                return 0.0; // magic number
            else
                return M_PI;
        }
        return acos(num/den);
    }
    inline void rot2d(double x, double y, double th, double &rx, double &ry)
    {
        rx = x*cos(th) - y*sin(th);
        ry = x*sin(th) + y*cos(th);
    }
private:
    // Each different map may have translation and rotation (orientation)
    // co-ordinates managed by user
    double map_res; // spetial resolution of the map
    duration_c time_res;
    boost::circular_buffer<TimeSlice<T> > time_circle;
    time_pt curr_time;
    void auto_timer()
    {
        duration_c tr = time_res;
        g_thread = std::thread(
                [tr, this] () {
                while (this->is_auto_step_time_on()) {
                std::this_thread::sleep_for(tr);
                this->step_time_unit(); } });
    }
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
