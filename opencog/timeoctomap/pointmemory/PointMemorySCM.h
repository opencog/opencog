/*
 * PointMemorySCM.h
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Mandeep Singh Bhatia <https://github.com/yantrabuddhi>
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

#ifndef _OPENCOG_POINT_MEM_SCM_H
#define _OPENCOG_POINT_MEM_SCM_H


#include <map>
#include <vector>

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/timeoctomap/TimeOctomap.h>


using namespace std;

namespace opencog
{
namespace ato
{
class PointMemorySCM
{
private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

public:
    //create a map
    bool create_map(const string& map_name,
                    double space_res_mtr,
                    int time_res_milli_sec,
                    int time_units);
    //get time resolution milli-sec
    int get_time_res(const string& map_name);
    //get space resolution meters
    double get_space_res(const string& map_name);
    //get time units
    int get_time_units(const string& map_name);
    //step time before adding atoms
    void step_time_unit(const string& map_name);
    //add an atom at location on current time step
    bool map_ato(const string& map_name,Handle ato,double x,double y,double z);
    //get time of first atom in past elapsed time
    Handle get_first_ato(const string& map_name,Handle ato,int elapse);
    //get time of last atom in past elapsed time
    Handle get_last_ato(const string& map_name,Handle ato,int elapse);
    //get atom at location
    Handle get_at_loc_ato(const string& map_name,double x,double y,double z);
    //get atom at location in elapsed past
    Handle get_past_loc_ato(const string& map_name,int elapse,
                            double x,double y,double z);
    //get locations of atom in current time
    Handle get_locs_ato(const string& map_name,Handle);//listlink atLocationLink
    //AtLocationLink
    //   Atom
    //   ListLink
    //     ConceptNode "map name"
    //     ListLink
    //       NumberNode x
    //       NumberNode y
    //       NumberNode z

    //get locations of atom in elapsed past
    Handle get_past_locs_ato(const string& map_name,Handle ato,int elapse);

    //AtTimeLink
    //  TimeNode "Date Time millisec"
    //  Atom
    // get time points of atom occuring at a location
    Handle get_elapse_list_at_loc_ato(const string& map_name, Handle ato,
              double x,double y,double z);//listlink atTimeLink
    //get time points of atom occuring in map
    Handle get_elapse_list_ato(const string& map_name,Handle ato);//listlink atTimeLink
    //remove atom from location at currrent time
    bool remove_location_ato(const string& map_name,double x,double y,double z);
    //remove atom from location at elapsed past time
    bool remove_past_location_ato(const string& map_name,int elapse,
         double x,double y,double z);
    //remove all specific atoms from map at current time
    void remove_curr_ato(const string& map_name,Handle ato);
    //remove all specific atoms from map in elapsed past 
    void remove_past_ato(const string& map_name,Handle ato,int elapse);
    //remove all specific atoms in all time points and all locations
    void remove_all_ato(const string& map_name,Handle ato);
    
    ////spatial query api
    
private:
    map<string,TimeOctomap*> tsa;
    
public:
    PointMemorySCM();
    ~PointMemorySCM();
};
}
}
extern "C" {
void opencog_ato_pointmem_init(void);
};
#endif
