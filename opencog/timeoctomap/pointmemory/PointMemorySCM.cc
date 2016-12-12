/*
 * PointMemorySCM.cc
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

#include "PointMemorySCM.h"
#include <opencog/guile/SchemePrimitive.h>
#include <iomanip>
#include <opencog/spacetime/atom_types.h>

using namespace std;
using namespace opencog::ato;
using namespace opencog;

/**
 * The constructor for PointMemorySCM.
 */
PointMemorySCM::PointMemorySCM()
{
    static bool is_init = false;
    if (is_init) return;
    is_init = true;
    scm_with_guile(init_in_guile, this);
}

PointMemorySCM::~PointMemorySCM()
{
    for (auto& x: tsa) delete x.second;
}
/**
 * Init function for using with scm_with_guile.
 *
 * Creates the PointMemorySCM scheme module and uses it by default.
 *
 * @param self   pointer to the PointMemorySCM object
 * @return       null
 */
void* PointMemorySCM::init_in_guile(void* self)
{
    scm_c_define_module("opencog ato pointmem", init_in_module, self);
    scm_c_use_module("opencog ato pointmem");
    return NULL;
}

/**
 * The main function for defining stuff in the PointMemorySCM scheme module.
 *
 * @param data   pointer to the PointMemorySCM object
 */
void PointMemorySCM::init_in_module(void* data)
{
    PointMemorySCM* self = (PointMemorySCM*) data;
    self->init();
}

/**
 * The main init function for the PointMemorySCM object.
 */
void PointMemorySCM::init()
{
#ifdef HAVE_GUILE
    define_scheme_primitive("create-map", &PointMemorySCM::create_map, this, "ato pointmem");// b_sdii
    define_scheme_primitive("get-time-res", &PointMemorySCM::get_time_res, this, "ato pointmem");// i_s
    define_scheme_primitive("get-space-res", &PointMemorySCM::get_space_res, this, "ato pointmem");// d_s
    define_scheme_primitive("get-time-units", &PointMemorySCM::get_time_units, this, "ato pointmem");// i_s
    define_scheme_primitive("step-time-unit", &PointMemorySCM::step_time_unit, this, "ato pointmem");// v_s
    define_scheme_primitive("auto-step-time-on", &PointMemorySCM::auto_step_time_on, this, "ato pointmem");// v_s
    define_scheme_primitive("auto-step-time-off", &PointMemorySCM::auto_step_time_off, this, "ato pointmem");// v_s
    define_scheme_primitive("is-auto-step-on", &PointMemorySCM::is_auto_step_on, this, "ato pointmem");// i_s
    define_scheme_primitive("map-ato", &PointMemorySCM::map_ato, this, "ato pointmem");// b_shddd
    define_scheme_primitive("get-first-ato", &PointMemorySCM::get_first_ato, this, "ato pointmem");// h_shi
    define_scheme_primitive("get-last-ato", &PointMemorySCM::get_last_ato, this, "ato pointmem");// h_shi
    define_scheme_primitive("get-first-locs-ato", &PointMemorySCM::get_first_locs_ato, this, "ato pointmem");// h_shi
    define_scheme_primitive("get-last-locs-ato", &PointMemorySCM::get_last_locs_ato, this, "ato pointmem");// h_shi
    define_scheme_primitive("get-at-loc-ato", &PointMemorySCM::get_at_loc_ato, this, "ato pointmem");// h_sddd
    define_scheme_primitive("get-past-loc-ato", &PointMemorySCM::get_past_loc_ato, this, "ato pointmem");// h_siddd
    define_scheme_primitive("get-locs-ato", &PointMemorySCM::get_locs_ato, this, "ato pointmem");// h_sh
    define_scheme_primitive("get-past-locs-ato", &PointMemorySCM::get_past_locs_ato, this, "ato pointmem");// h_shi
    define_scheme_primitive("get-elapse-list-at-loc-ato", &PointMemorySCM::get_elapse_list_at_loc_ato, this, "ato pointmem");// h_shddd
    define_scheme_primitive("get-elapse-list-ato", &PointMemorySCM::get_elapse_list_ato, this, "ato pointmem");// h_sh
    define_scheme_primitive("remove-location-ato", &PointMemorySCM::remove_location_ato, this, "ato pointmem");// b_sddd
    define_scheme_primitive("remove-past-location-ato", &PointMemorySCM::remove_past_location_ato, this, "ato pointmem");// b_siddd
    define_scheme_primitive("remove-curr-ato", &PointMemorySCM::remove_curr_ato, this, "ato pointmem");// v_sh
    define_scheme_primitive("remove-past-ato", &PointMemorySCM::remove_past_ato, this, "ato pointmem");// v_shi
    define_scheme_primitive("remove-all-ato", &PointMemorySCM::remove_all_ato, this, "ato pointmem");// v_sh

    // spatial query
    define_scheme_primitive("get-dist-ato", &PointMemorySCM::get_distance_between, this, "ato pointmem");// d_shhi
    define_scheme_primitive("get-angular-nearness-ato", &PointMemorySCM::get_angular_nearness, this, "ato pointmem");// i_shhhi
    define_scheme_primitive("get-right-left-of-ato", &PointMemorySCM::get_target_is_right_left, this, "ato pointmem");// i_shhhi
    define_scheme_primitive("get-above-below-of-ato", &PointMemorySCM::get_target_is_above_below, this, "ato pointmem");// i_shhhi
    define_scheme_primitive("get-front-back-of-ato", &PointMemorySCM::get_target_is_front_back, this, "ato pointmem");// i_shhhi
#endif
}

bool PointMemorySCM::create_map(const string& name,
                                double space_res_mtr,
                                int time_res_milli_sec,
                                int time_units)
{
    if (name.length() < 1)
        return false;

    // Reject if time units < 1
    if (time_units < 1)
        return false;

    // Reject if time res, space_res <= 0
    if (time_res_milli_sec <= 0 || space_res_mtr <= 0.0)
        return false;

    // Reject if name already exists
    std::map<string, TimeOctomap*>::iterator it = tsa.find(name);
    if (it ! = tsa.end())
        return false;

    tsa[name] = new TimeOctomap(time_units, space_res_mtr, std::chrono::milliseconds(time_res_milli_sec));
    return true;
}
// add point clouds later

int PointMemorySCM::get_time_res(const string& map_name)
{
    duration_c dr = tsa[map_name]->get_time_resolution();
    return chrono::duration_cast<chrono::milliseconds>(dr).count();
}

double PointMemorySCM::get_space_res(const string& map_name)
{
    return tsa[map_name]->get_space_resolution();
}

int PointMemorySCM::get_time_units(const string& map_name)
{
    return tsa[map_name]->get_time_units();
}

void PointMemorySCM::step_time_unit(const string& map_name)
{
    tsa[map_name]->step_time_unit();
}

void PointMemorySCM::auto_step_time_on(const string& map_name)
{
    tsa[map_name]->auto_step_time(true);
}

void PointMemorySCM::auto_step_time_off(const string& map_name)
{
    tsa[map_name]->auto_step_time(false);
}

int PointMemorySCM::is_auto_step_on(const string& map_name)
{
    return (tsa[map_name]->is_auto_step_time_on()) ? 1 : 0;
}

bool PointMemorySCM::map_ato(const string& map_name, Handle ato,
                             double x, double y, double z)
{
    return tsa[map_name]->put_atom_at_current_time(point3d(x, y, z), ato);
}

// Tag the atom `ato` with the timepoint `tp`, using an AtTimeLink
static Handle timestamp_tag_atom(const time_pt& tp, const Handle& ato)
{
    // XXX FIXME - there should be a TimeNode c++ class that will
    // store timestamps internally, and avoid or minimize this nasty
    // string conversion.
    std::time_t ttp = std::chrono::system_clock::to_time_t(tp);
    char buff[31];
    strftime(buff, 30, "%Y-%m-%d %H:%M:%S ", std::localtime(&ttp));
    std::string ts(buff);
    // time_since_epoch gives duration, duration to seconds and
    // milliseconds then subtract [millisec - sec to millisec]
    // add milli sec to ts
    long d_sec = chrono::duration_cast<chrono::seconds>(tp.time_since_epoch()).count();
    long d_mil = chrono::duration_cast<chrono::milliseconds>(tp.time_since_epoch()).count();
    long mil_diff = d_mil-d_sec * 1000;
    ts += to_string(mil_diff);
    return Handle(createLink(AT_TIME_LINK, Handle(createNode(TIME_NODE, ts)), ato));
}

Handle PointMemorySCM::get_first_ato(const string& map_name,
                                     Handle ato, int elapse)
{
    time_pt tpt, tp;
    if (not get_map_time(map_name, elapse, tpt))
        return UndefinedHandle;

    bool r = tsa[map_name]->get_oldest_time_elapse_atom_observed(ato, tpt, tp);
    if (not r)
        return UndefinedHandle;

    // Make and return atTimeLink
    return timestamp_tag_atom(tp, ato);
}

Handle PointMemorySCM::get_last_ato(const string& map_name,
                                    Handle ato, int elapse)
{
    time_pt tpt, tp;
    if (not get_map_time(map_name, elapse, tpt))
        return UndefinedHandle;

    bool r = tsa[map_name]->get_last_time_elapse_atom_observed(ato, tpt, tp);
    if (not r)
        return UndefinedHandle;

    // make and return atTimeLink
    return timestamp_tag_atom(tp, ato);
}

Handle PointMemorySCM::get_at_loc_ato(const string& map_name,
                                      double x, double y, double z)
{
    Handle ato;
    if (tsa[map_name]->get_atom_current_time_at_location(point3d(x, y, z), ato))
        return ato;
    return UndefinedHandle;
}

Handle PointMemorySCM::get_past_loc_ato(const string& map_name, int elapse,
                            double x, double y, double z)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return UndefinedHandle;
    Handle ato;
    if (tsa[map_name]->get_atom_at_time_by_location(tpt, point3d(x, y, z), ato))
        return ato;
    return UndefinedHandle;
}

// Convert a pointlist to a SetLink of AtLocationLink's
// Returns an empty SetLink if the pointlist is empty.
static Handle pointlist_to_atom(const point3d_list& pl)
{
    HandleSeq loc_links;
    for (const point3d& pt: pl)
    {
        loc_links.push_back(Handle(createLink(AT_LOCATION_LINK,
            Handle(createNode(OBJECT_NODE, map_name)),
            Handle(createLink(LIST_LINK, ato,
                Handle(createLink(LIST_LINK,
                    Handle(createNode(NUMBER_NODE, to_string(pt.x()))),
                    Handle(createNode(NUMBER_NODE, to_string(pt.y()))),
                    Handle(createNode(NUMBER_NODE, to_string(pt.z())))
                ))
            ))
        )));
    }
    return Handle(createLink(SET_LINK, loc_links));
}

Handle PointMemorySCM::get_first_locs_ato(const string& map_name,
                                          Handle ato, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return UndefinedHandle;

    point3d_list pl;
    tsa[map_name]->get_oldest_time_locations_atom_observed(ato, tpt, pl);
    return pointlist_to_atom(pl);
}


Handle PointMemorySCM::get_last_locs_ato(const string& map_name, Handle ato, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return UndefinedHandle;

    point3d_list pl;
    tsa[map_name]->get_last_locations_of_atom_observed(ato, tpt, pl);

    return pointlist_to_atom(pl);
}

Handle PointMemorySCM::get_locs_ato(const string& map_name, Handle ato)// listlink atLocationLink
{
    point3d_list pl = tsa[map_name]->get_locations_of_atom_occurence_now(ato);
    return pointlist_to_atom(pl);
}

Handle PointMemorySCM::get_past_locs_ato(const string& map_name, Handle ato, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return UndefinedHandle;
    point3d_list pl = tsa[map_name]->get_locations_of_atom_occurence_at_time(tpt, ato);
    return pointlist_to_atom(pl);
}

Handle PointMemorySCM::get_elapse_list_at_loc_ato(const string& map_name,
              Handle ato,
              double x, double y, double z)
{
    time_list tl = tsa[map_name]->get_times_of_atom_occurence_at_location(point3d(x, y, z), ato);

    HandleSeq LL;
    for (const time_pt& tp: tl)
        LL.push_back(timestamp_tag_atom(tp, ato));

    return opencog::Handle(createLink(SET_LINK, LL));
}

Handle PointMemorySCM::get_elapse_list_ato(const string& map_name, Handle ato)// listlink atTimeLink
{
    time_list tl = tsa[map_name]->get_times_of_atom_occurence_in_map(ato);
    HandleSeq LL;
    for (const time_pt& tp: tl)
        LL.push_back(timestamp_tag_atom(tp, ato));
    return opencog::Handle(createLink(SET_LINK, LL));
}

bool PointMemorySCM::remove_location_ato(const string& map_name, double x, double y, double z)
{
    return tsa[map_name]->remove_atom_at_current_time_by_location(point3d(x, y, z));
}

bool PointMemorySCM::remove_past_location_ato(const string& map_name, int elapse,
         double x, double y, double z)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return false;
    return tsa[map_name]->remove_atom_at_time_by_location(tpt, point3d(x, y, z));
}

void PointMemorySCM::remove_curr_ato(const string& map_name, Handle ato)
{
    tsa[map_name]->remove_atom_at_current_time(ato);
}

void PointMemorySCM::remove_past_ato(const string& map_name, Handle ato, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return;
    tsa[map_name]->remove_atom_at_time(tpt, ato);
}

void PointMemorySCM::remove_all_ato(const string& map_name, Handle ato)
{
    tsa[map_name]->remove_atom(ato);
}


bool
PointMemorySCM::get_map_time(const string& map_name, int elapse, time_pt& tpt)
{
    duration_c dr;
    if (!tsa[map_name]->get_current_time_range(tpt, dr))
        return false;
    dr = std::chrono::milliseconds(elapse);
    tpt -= dr;
    return true;
}
// Spatial query api assuming 1 atom in 1 map at 1 location.
// For multi-map support, need to add features to main API, to provide
// more raw data results -ve for unknown
double
PointMemorySCM::get_distance_between(const string& map_name, Handle ato1, Handle ato2, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return -1.0;
    return tsa[map_name]->get_distance_between(tpt, ato1, ato2);
}

// 2 = far, 1 = near, 0 = touching, -1 = unknown
int
PointMemorySCM::get_angular_nearness(const string& map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return -1.0;
    return tsa[map_name]->get_angular_nearness(tpt, ato_obs, ato_tgt, ato_ref);
}

// 2 = right, 1 = left, 0 = aligned, -1 = unknown
int
PointMemorySCM::get_target_is_right_left(const string& map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return -1.0;
    point3d res = tsa[map_name]->get_spatial_relations(tpt, ato_obs, ato_tgt, ato_ref);
    return lround(res.y());
}

// 2 = above, 1 = below, 0 = aligned, -1 = unknown
int
PointMemorySCM::get_target_is_above_below(const string& map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return -1.0;
    point3d res = tsa[map_name]->get_spatial_relations(tpt, ato_obs, ato_tgt, ato_ref);
    return lround(res.z());
}

// 2 = ahead/front, 1 = behind/back, 0 = aligned, -1 = unknown
int
PointMemorySCM::get_target_is_front_back(const string& map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
    time_pt tpt;
    if (!get_map_time(map_name, elapse, tpt))
        return -1.0;
    point3d res = tsa[map_name]->get_spatial_relations(tpt, ato_obs, ato_tgt, ato_ref);
    return lround(res.x());
}

void opencog_ato_pointmem_init(void)
{
    static PointMemorySCM pm;
}
