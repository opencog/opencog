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


#include <iomanip>
#include <map>
#include <vector>

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/timeoctomap/TimeOctomap.h>

namespace opencog
{
class PointMemorySCM
{
private:
	static void* init_in_guile(void*);
	static void init_in_module(void*);
	void init(void);

	// Check if the named map exists.
	void have_map(const Handle&);

public:
	// Create a map
	Handle create_map(Handle map_name, Handle resolution);

	// Get time resolution milli-sec
	Handle get_time_res(Handle);
	// Get space resolution meters
	Handle get_space_res(Handle);
	// Get time units
	Handle get_time_units(Handle);
	// Step time before adding atoms
	void step_time_unit(Handle);
	void auto_step_time_on(Handle);
	void auto_step_time_off(Handle);

	TruthValuePtr is_auto_step_on(Handle);
	// Add an atom at location on current time step
	void map_ato(Handle map, Handle ato, Handle loc);

	// Get time of first atom in past elapsed time
	Handle get_first_time(Handle, Handle ato, Handle elapse);
	// Get time of last atom in past elapsed time
	Handle get_last_time(Handle, Handle ato, Handle elapse);

	// Get atom at location
	Handle get_at_loc_ato(Handle, Handle);
	// Get atom at location in elapsed past
	Handle get_past_loc_ato(Handle, Handle, Handle);

	// Get location of atom at current time
	Handle get_locs_ato(Handle, Handle);//listlink atLocationLink
	//AtLocationLink
	//   Atom
	//   ListLink
	//	 ConceptNode "map name"
	//	 ListLink
	//	   NumberNode x
	//	   NumberNode y
	//	   NumberNode z

	// Get locations of atom in elapsed past
	Handle get_past_locs_ato(Handle, Handle ato, Handle elapse);
	Handle get_first_location(Handle, Handle ato, Handle elapse);
	Handle get_last_location(Handle, Handle ato, Handle elapse);
	//AtTimeLink
	//  TimeNode "Date Time millisec"
	//  Atom
	// Get time points of atom occuring at a location
	Handle get_elapse_list_at_loc_ato(Handle, Handle ato,
			  Handle loc);//listlink atTimeLink
	// Get time points of atom occuring in map
	Handle get_timeline(Handle, Handle ato);//listlink atTimeLink
	// Remove atom from location at currrent time
	bool remove_location_ato(Handle, Handle loc);
	// Remove atom from location at elapsed past time
	bool remove_past_location_ato(Handle, Handle, Handle);
	// Remove all specific atoms from map at current time
	void remove_curr_ato(Handle, Handle ato);
	// Remove all specific atoms from map in elapsed past
	void remove_past_ato(Handle, Handle ato, Handle elapse);
	// Remove all specific atoms in all time points and all locations
	void remove_all_ato(Handle, Handle ato);

	////spatial query api assuming 1 ato in 1 map at 1 location.
	//for multi-map need to add features to main api to provide more raw data results
	//-ve for unknown
	double get_distance_between(Handle, Handle ato1, Handle ato2, int elapse);
	// 2=far, 1=near, 0=touching, -1=unknown
	int get_angular_nearness(Handle, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse);
	// 2=right, 1=left, 0=aligned, -1=unknown
	int get_target_is_right_left(Handle, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse);
	// 2=above, 1=below, 0=aligned, -1=unknown
	int get_target_is_above_below(Handle, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse);
	// 2=ahead, 1=behind, 0=aligned, -1=unknown
	int get_target_is_front_back(Handle, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse);
private:
	std::map<std::string, TimeOctomap*> tsa;
	time_pt get_map_time(Handle, int elapse);
public:
	PointMemorySCM();
	~PointMemorySCM();
};
}

extern "C" {
void opencog_ato_pointmem_init(void);
};

using namespace std;
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
 * @return	   null
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
	define_scheme_primitive("get-first-ato", &PointMemorySCM::get_first_time, this, "ato pointmem");// h_shi
	define_scheme_primitive("get-last-ato", &PointMemorySCM::get_last_time, this, "ato pointmem");// h_shi
	define_scheme_primitive("get-first-locs-ato", &PointMemorySCM::get_first_location, this, "ato pointmem");// h_shi

	// -----------------------------------------
	define_scheme_primitive("get-last-locs-ato", &PointMemorySCM::get_last_location, this, "ato pointmem");// h_shi
	define_scheme_primitive("get-at-loc-ato", &PointMemorySCM::get_at_loc_ato, this, "ato pointmem");// h_sddd
	define_scheme_primitive("get-past-loc-ato", &PointMemorySCM::get_past_loc_ato, this, "ato pointmem");// h_siddd
	define_scheme_primitive("get-locs-ato", &PointMemorySCM::get_locs_ato, this, "ato pointmem");// h_sh
	define_scheme_primitive("get-past-locs-ato", &PointMemorySCM::get_past_locs_ato, this, "ato pointmem");// h_shi
	define_scheme_primitive("get-elapse-list-at-loc-ato", &PointMemorySCM::get_elapse_list_at_loc_ato, this, "ato pointmem");// h_shddd
	define_scheme_primitive("get-elapse-list-ato", &PointMemorySCM::get_timeline, this, "ato pointmem");// h_sh
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

Handle PointMemorySCM::create_map(Handle name, Handle resolution)
{
	HandleSeq& hs->getOutgoingSet();

	// XXX FIXME -- verify that these are NumberNodes.
	double space_res_mtr = atof(hs[0]->getName());
	int time_res_milli_sec = atoi(hs[1]->getName());
	int time_units = atoi(hs[2]->getName());

	// Reject if time units < 1
	if (time_units < 1)
		throw InvalidParamException(TRACE_INFO,
			 "Expecting positive time unit");

	// Reject if time res, space_res <= 0
	if (time_res_milli_sec <= 0 || space_res_mtr <= 0.0)
		throw InvalidParamException(TRACE_INFO,
			 "Expecting positive spatial resolution");

	if (name->getName().length() < 1)
		throw InvalidParamException(TRACE_INFO,
			 "Invalid map name");

	// Reject if name already exists
	if (tsa.find(map_name->getName()) != tsa.end())
		throw InvalidParamException(TRACE_INFO, "Map already exists");

	tsa[name] = new TimeOctomap(time_units, space_res_mtr, std::chrono::milliseconds(time_res_milli_sec));
	return name;
}
// add point clouds later

void PointMemorySCM::have_map(Handle map_name)
{
	if (tsa.find(map_name->getName()) == tsa.end())
		throw InvalidParamException(TRACE_INFO, "Map does not exist");
}

Handle PointMemorySCM::get_time_res(Handle map_name)
{
	have_map(map_name);
	duration_c dr = tsa[map_name->getName()]->get_time_resolution();
	return Handle(createNumberNode(
		chrono::duration_cast<chrono::milliseconds>(dr).count()));
}

Handle PointMemorySCM::get_space_res(Handle map_name)
{
	have_map(map_name);
	return Handle(createNumberNode(
		tsa[map_name->getName()]->get_space_resolution()));
}

Handle PointMemorySCM::get_time_units(Handle map_name)
{
	have_map(map_name);
	return Handle(createNumberNode(
		tsa[map_name->getName()]->get_time_units()));
}

void PointMemorySCM::step_time_unit(Handle map_name)
{
	have_map(map_name);
	tsa[map_name->getName()]->step_time_unit();
}

void PointMemorySCM::auto_step_time_on(Handle map_name)
{
	have_map(map_name);
	tsa[map_name->getName()]->auto_step_time(true);
}

void PointMemorySCM::auto_step_time_off(Handle map_name)
{
	have_map(map_name);
	tsa[map_name->getName()]->auto_step_time(false);
}

TruthValuePtr PointMemorySCM::is_auto_step_on(Handle map_name)
{
	have_map(map_name);
	return (tsa[map_name->getName()]->is_auto_step_time_on()) ?
		TruthValue::TRUE_TV() : TruthValue::FALSE_TV();
}

void PointMemorySCM::map_ato(Handle map_name, Handle ato, Handle loc)
{
	have_map(map_name);
	// loc should be a ListLink of three NumberNodes.
	HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();
	tsa[map_name->getName()]->insert_atom(point3d(x, y, z), ato);
}

// Tag the atom `ato` with the timepoint `tp`, using an AtTimeLink
static Handle timestamp_tag_atom(const time_pt& tp, Handle ato)
{
	// XXX FIXME - there should be a TimeNode C++ class that will
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

Handle PointMemorySCM::get_first_time(Handle map_name,
									 Handle ato, Handle helapse)
{
	have_map(map_name);

	int elapse = atoi(helapse->getName());
	time_pt tpt = get_map_time(map_name, elapse);

	time_pt tp;
	bool r = tsa[map_name->getName()]->get_oldest_time_elapse_atom_observed(ato, tpt, tp);
	if (not r) return Handle(); // XXX should this throw instead?

	// Make and return atTimeLink
	return timestamp_tag_atom(tp, ato);
}

Handle PointMemorySCM::get_last_time(Handle map_name,
									 Handle ato, Handle helapse)
{
	have_map(map_name);

	int elapse = atoi(helapse->getName());
	time_pt tpt = get_map_time(map_name, elapse);

	time_pt tp;
	bool r = tsa[map_name->getName()]->get_last_time_elapse_atom_observed(ato, tpt, tp);
	if (not r)
		return UndefinedHandle; // XXX should this throw instead?

	// make and return atTimeLink
	return timestamp_tag_atom(tp, ato);
}

Handle PointMemorySCM::get_at_loc_ato(Handle map_name,
                             Handle loc)
{
	have_map(map_name);
	// loc should be a ListLink of three NumberNodes.
	HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();
	return tsa[map_name->getName()]->get_atom_at_location(point3d(x, y, z));
}

Handle PointMemorySCM::get_past_loc_ato(Handle map_name,
                                        Handle loc,
                                        Handle helapse)
{
	have_map(map_name);
	// loc should be a ListLink of three NumberNodes.
	HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();
	int elapse = atoi(helapse->getName());

	time_pt tpt = get_map_time(map_name, elapse);
	return tsa[map_name->getName()]->get_atom_at_time_by_location(tpt, point3d(x, y, z));
}

// Tag an atom with SetLink of locations (AtLocationLink)
// Returns an empty SetLink if the pointlist is empty.
static Handle tag_atom_with_locs(Handle map_name,
								 Handle ato,
								 const point3d_list& pl)
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
	return Handle(createLink(loc_links, SET_LINK));
}

Handle PointMemorySCM::get_first_location(Handle map_name,
										  Handle ato, Handle helapse)
{
	have_map(map_name);

	int elapse = atoi(helapse->getName());
	time_pt tpt = get_map_time(map_name, elapse);
	point3d_list pl = tsa[map_name->getName()]->get_oldest_locations(ato, tpt);
	return tag_atom_with_locs(map_name, ato, pl);
}


Handle PointMemorySCM::get_last_location(Handle map_name,
										  Handle ato, Handle helapse)
{
	have_map(map_name);

	int elapse = atoi(helapse->getName());
	time_pt tpt = get_map_time(map_name, elapse);
	point3d_list pl = tsa[map_name->getName()]->get_newest_locations(ato, tpt);
	return tag_atom_with_locs(map_name, ato, pl);
}

Handle PointMemorySCM::get_locs_ato(Handle map_name, Handle ato)
{
	have_map(map_name);
	point3d_list pl = tsa[map_name->getName()]->get_locations_of_atom(ato);
	return tag_atom_with_locs(map_name, ato, pl);
}

Handle PointMemorySCM::get_past_locs_ato(Handle map_name,
                                         Handle ato, Handle helapse)
{
	have_map(map_name);

	int elapse = atoi(helapse->getName());
	time_pt tpt = get_map_time(map_name, elapse);
	point3d_list pl = tsa[map_name->getName()]->get_locations_of_atom_at_time(tpt, ato);
	return tag_atom_with_locs(map_name, ato, pl);
}

Handle PointMemorySCM::get_elapse_list_at_loc_ato(Handle map_name,
			  Handle ato,
			  double x, double y, double z)
{
	if (not have_map(map_name)) return Handle();
	time_list tl = tsa[map_name->getName()]->get_times_of_atom_occurence_at_location(point3d(x, y, z), ato);

	HandleSeq LL;
	for (const time_pt& tp: tl)
		LL.push_back(timestamp_tag_atom(tp, ato));

	return opencog::Handle(createLink(LL, SET_LINK));
}

// Get the timeline of the atom.  That is, get a sequence of
// time-points, at which the atom is in the map.
Handle PointMemorySCM::get_timeline(Handle map_name, Handle ato)
{
	if (not have_map(map_name)) return Handle();
	time_list tl = tsa[map_name->getName()]->get_timeline(ato);
	HandleSeq LL;
	for (const time_pt& tp: tl)
		LL.push_back(timestamp_tag_atom(tp, ato));
	return opencog::Handle(createLink(LL, SET_LINK));
}

bool PointMemorySCM::remove_location_ato(Handle map_name, double x, double y, double z)
{
	if (not have_map(map_name)) return true;
	tsa[map_name->getName()]->remove_atoms_at_location(point3d(x, y, z));
	return true;
}

bool PointMemorySCM::remove_past_location_ato(Handle map_name, int elapse,
		 double x, double y, double z)
{
	if (not have_map(map_name)) return true;
	time_pt tpt = get_map_time(map_name, elapse);
	tsa[map_name->getName()]->remove_atom_at_time_by_location(tpt, point3d(x, y, z));
	return true;
}

void PointMemorySCM::remove_curr_ato(Handle map_name, Handle ato)
{
	if (not have_map(map_name)) return;
	tsa[map_name->getName()]->remove_atom_at_current_time(ato);
}

void PointMemorySCM::remove_past_ato(Handle map_name, Handle ato, int elapse)
{
	if (not have_map(map_name)) return;
	time_pt tpt = get_map_time(map_name, elapse);
	tsa[map_name->getName()]->remove_atom_at_time(tpt, ato);
}

void PointMemorySCM::remove_all_ato(Handle map_name, Handle ato)
{
	if (not have_map(map_name)) return;
	tsa[map_name->getName()]->remove_atom(ato);
}


time_pt PointMemorySCM::get_map_time(Handle map_name, int elapse)
{
	if (not have_map(map_name)) return time_pt();
	return tsa[map_name->getName()]->get_current_time() - std::chrono::milliseconds(elapse);
}
// Spatial query api assuming 1 atom in 1 map at 1 location.
// elapse is in milliseconds (in the past)
// For multi-map support, need to add features to main API, to provide
// more raw data results -ve for unknown
double
PointMemorySCM::get_distance_between(Handle map_name, Handle ato1, Handle ato2, int elapse)
{
	if (not have_map(map_name)) return 0.0;
	time_pt tpt = get_map_time(map_name, elapse);
	return tsa[map_name->getName()]->get_distance_between(tpt, ato1, ato2);
}

// 2 = far, 1 = near, 0 = touching
int
PointMemorySCM::get_angular_nearness(Handle map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
	if (not have_map(map_name)) return -1;
	time_pt tpt = get_map_time(map_name, elapse);
	return tsa[map_name->getName()]->get_angular_nearness(tpt, ato_obs, ato_tgt, ato_ref);
}

// 2 = right, 1 = left, 0 = aligned
int
PointMemorySCM::get_target_is_right_left(Handle map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
	if (not have_map(map_name)) return -1;
	time_pt tpt = get_map_time(map_name, elapse);
	point3d res = tsa[map_name->getName()]->get_spatial_relations(tpt, ato_obs, ato_tgt, ato_ref);
	return lround(res.y());
}

// 2 = above, 1 = below, 0 = aligned
int
PointMemorySCM::get_target_is_above_below(Handle map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
	if (not have_map(map_name)) return -1;
	time_pt tpt = get_map_time(map_name, elapse);
	point3d res = tsa[map_name->getName()]->get_spatial_relations(tpt, ato_obs, ato_tgt, ato_ref);
	return lround(res.z());
}

// 2 = ahead/front, 1 = behind/back, 0 = aligned
int
PointMemorySCM::get_target_is_front_back(Handle map_name, Handle ato_obs, Handle ato_tgt, Handle ato_ref, int elapse)
{
	if (not have_map(map_name)) return -1;
	time_pt tpt = get_map_time(map_name, elapse);
	point3d res = tsa[map_name->getName()]->get_spatial_relations(tpt, ato_obs, ato_tgt, ato_ref);
	return lround(res.x());
}

void opencog_ato_pointmem_init(void)
{
	static PointMemorySCM pm;
}
