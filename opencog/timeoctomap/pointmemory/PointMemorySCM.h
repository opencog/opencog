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
    //bool .. handle float float float
public:
    bool create_map(string map_name,
                    double space_res_mtr,
                    int time_res_milli_sec,
                    int time_units);
    int get_time_res(string map_name);
    double get_space_res(string map_name);
    int get_time_units(string map_name);

    void step_time_unit(string map_name);
    bool map_ato(string map_name,Handle,double,double,double);
    Handle get_first_ato(string map_name,Handle,int elapse);
    Handle get_last_ato(string map_name,Handle,int elapse);
    Handle get_at_loc_ato(string map_name,double,double,double);
    Handle get_past_loc_ato(string map_name,int elapse,
                            double,double,double);
    Handle get_locs_ato(string map_name,Handle);//listlink atLocationLink
    //AtLocationLink
    //   Atom
    //   ListLink
    //     ConceptNode "map name"
    //     ListLink
    //       NumberNode x
    //       NumberNode y
    //       NumberNode z
    Handle get_past_locs_ato(string map_name,Handle,int elapse);
    Handle get_elapse_list_at_loc_ato(string map_name, Handle,
              double,double,double);//listlink atTimeLink
    Handle get_elapse_list_ato(string map_name,Handle);//listlink atTimeLink
    bool remove_location_ato(string map_name,double,double,double);
    bool remove_past_location_ato(string map_name,int elapse,
         double,double,double);
    void remove_curr_ato(string map_name,Handle);
    void remove_past_ato(string map_name,Handle,int elapse);
    void remove_all_ato(string map_name,Handle);
    //list .. handle float float
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
