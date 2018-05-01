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

#include <math.h>

#include <iomanip>
#include <map>
#include <vector>

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atoms/core/NumberNode.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/spacetime/atom-types/atom_types.h>
#include <opencog/spacetime/octomap/TimeOctomap.h>

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

	std::map<Handle, TimeOctomap<Handle>*> tsa;
	time_pt get_map_time(const Handle& map_name, const Handle& elapse);

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
	Handle map_ato(Handle map, Handle ato, Handle loc);

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
	//  Atom
	//  TimeNode "Date Time millisec"
	// Get time points of atom occuring at a location
	Handle get_elapse_list_at_loc_ato(Handle, Handle ato,
	                                  Handle loc);//listlink atTimeLink
	// Get time points of atom occuring in map
	Handle get_timeline(Handle, Handle ato);//listlink atTimeLink
	// Remove atom from location at currrent time
	Handle remove_location_ato(Handle, Handle loc);
	// Remove atom from location at elapsed past time
	Handle remove_past_location_ato(Handle, Handle, Handle);
	// Remove all specific atoms from map at current time
	Handle remove_curr_ato(Handle, Handle ato);
	// Remove all specific atoms from map in elapsed past
	Handle remove_past_ato(Handle, Handle ato, Handle elapse);
	// Remove all specific atoms in all time points and all locations
	Handle remove_all_ato(Handle, Handle ato);

	////spatial query api assuming 1 ato in 1 map at 1 location.
	Handle get_distance_between(Handle, Handle list, Handle elapse);
	// 2=far, 1=near, 0=touching, -1=unknown
	Handle get_angular_nearness(Handle, Handle, Handle elapse);
	// 2=right, 1=left, 0=aligned, -1=unknown
	Handle get_target_is_right_left(Handle, Handle, Handle elapse);
	// 2=above, 1=below, 0=aligned, -1=unknown
	Handle get_target_is_above_below(Handle, Handle, Handle elapse);
	// 2=ahead, 1=behind, 0=aligned, -1=unknown
	Handle get_target_is_front_back(Handle, Handle, Handle elapse);

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
	scm_c_define_module("opencog pointmem", init_in_module, self);
	scm_c_use_module("opencog pointmem");
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
	define_scheme_primitive("cog-pointmem-create-map", &PointMemorySCM::create_map, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-time-res", &PointMemorySCM::get_time_res, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-space-res", &PointMemorySCM::get_space_res, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-time-units", &PointMemorySCM::get_time_units, this, "pointmem");
	define_scheme_primitive("cog-pointmem-step-time-unit", &PointMemorySCM::step_time_unit, this, "pointmem");
	define_scheme_primitive("cog-pointmem-auto-step-time-on", &PointMemorySCM::auto_step_time_on, this, "pointmem");
	define_scheme_primitive("cog-pointmem-auto-step-time-off", &PointMemorySCM::auto_step_time_off, this, "pointmem");
	define_scheme_primitive("cog-pointmem-is-auto-step-on", &PointMemorySCM::is_auto_step_on, this, "pointmem");
	define_scheme_primitive("cog-pointmem-map-atom", &PointMemorySCM::map_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-first-atom", &PointMemorySCM::get_first_time, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-last-atom", &PointMemorySCM::get_last_time, this, "pointmem");

	// -----------------------------------------
	define_scheme_primitive("cog-pointmem-get-first-locs-of-atom", &PointMemorySCM::get_first_location, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-last-locs-of-atom", &PointMemorySCM::get_last_location, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-at-loc-atom", &PointMemorySCM::get_at_loc_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-past-loc-of-atom", &PointMemorySCM::get_past_loc_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-locs-of-atom", &PointMemorySCM::get_locs_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-past-locs-of-atom", &PointMemorySCM::get_past_locs_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-elapsed-list-at-loc", &PointMemorySCM::get_elapse_list_at_loc_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-elapsed-list", &PointMemorySCM::get_timeline, this, "pointmem");
	define_scheme_primitive("cog-pointmem-remove-location-of-atom", &PointMemorySCM::remove_location_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-remove-past-location-of-atom", &PointMemorySCM::remove_past_location_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-remove-curr-atom", &PointMemorySCM::remove_curr_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-remove-past-atom", &PointMemorySCM::remove_past_ato, this, "pointmem");
	define_scheme_primitive("cog-pointmem-remove-all-atoms", &PointMemorySCM::remove_all_ato, this, "pointmem");

	// spatial query
	define_scheme_primitive("cog-pointmem-get-dist-to-atom", &PointMemorySCM::get_distance_between, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-angular-nearness-to-atom", &PointMemorySCM::get_angular_nearness, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-right-left-of-atom", &PointMemorySCM::get_target_is_right_left, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-above-below-of-atom", &PointMemorySCM::get_target_is_above_below, this, "pointmem");
	define_scheme_primitive("cog-pointmem-get-front-back-of-atom", &PointMemorySCM::get_target_is_front_back, this, "pointmem");
#endif
}

double get_float(const Handle& h, const char * msg)
{
	NumberNodePtr nn = NumberNodeCast(h);
	if (nullptr == nn)
		throw InvalidParamException(TRACE_INFO,
			 "Expecting a number for %s, got %s", msg, h->to_string().c_str());
	return nn->get_value();
}

int get_int(const Handle& h, const char * msg)
{
	return (int) round(get_float(h, msg));
}

Handle PointMemorySCM::create_map(Handle map_name, Handle resolution)
{
	const HandleSeq& hs = resolution->getOutgoingSet();

	double space_res_mtr = get_float(hs[0], "space resolution");
	int time_res_milli_sec = get_int(hs[1], "time resolution (millisecs)");
	int time_units = get_int(hs[2], "time units");

	// Reject if time units < 1
	if (time_units < 1)
		throw InvalidParamException(TRACE_INFO,
			 "Expecting positive time unit");

	// Reject if time res, space_res <= 0
	if (time_res_milli_sec <= 0 || space_res_mtr <= 0.0)
		throw InvalidParamException(TRACE_INFO,
			 "Expecting positive spatial resolution");

	// The map MUST be disambiguated, else lookups will fail.
	if (nullptr == map_name->getAtomSpace())
		throw InvalidParamException(TRACE_INFO,
			 "Invalid map");

	// Reject if name already exists
	if (tsa.find(map_name) != tsa.end())
		throw InvalidParamException(TRACE_INFO, "Map already exists");

	tsa[map_name] = new TimeOctomap<Handle>(time_units, space_res_mtr,
	                  std::chrono::milliseconds(time_res_milli_sec));
	return map_name;
}
// add point clouds later

void PointMemorySCM::have_map(const Handle& map_name)
{
	if (tsa.find(map_name) == tsa.end())
		throw InvalidParamException(TRACE_INFO, "Map does not exist");
}

Handle PointMemorySCM::get_time_res(Handle map_name)
{
	have_map(map_name);
	duration_c dr = tsa[map_name]->get_time_resolution();
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(
		chrono::duration_cast<chrono::milliseconds>(dr).count())));
}

Handle PointMemorySCM::get_space_res(Handle map_name)
{
	have_map(map_name);
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(
		tsa[map_name]->get_space_resolution())));
}

Handle PointMemorySCM::get_time_units(Handle map_name)
{
	have_map(map_name);
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(
		tsa[map_name]->get_time_units())));
}

void PointMemorySCM::step_time_unit(Handle map_name)
{
	have_map(map_name);
	tsa[map_name]->step_time_unit();
}

void PointMemorySCM::auto_step_time_on(Handle map_name)
{
	have_map(map_name);
	tsa[map_name]->auto_step_time(true);
}

void PointMemorySCM::auto_step_time_off(Handle map_name)
{
	have_map(map_name);
	tsa[map_name]->auto_step_time(false);
}

TruthValuePtr PointMemorySCM::is_auto_step_on(Handle map_name)
{
	have_map(map_name);
	return (tsa[map_name]->is_auto_step_time_on()) ?
		TruthValue::TRUE_TV() : TruthValue::FALSE_TV();
}

Handle PointMemorySCM::map_ato(Handle map_name, Handle ato, Handle loc)
{
	have_map(map_name);
	// loc should be a ListLink of three NumberNodes.
	const HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();
	tsa[map_name]->insert_atom(point3d(x, y, z), ato);
	return ato;
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
	return Handle(createLink(AT_TIME_LINK, ato, Handle(createNode(TIME_NODE, ts))));
}

Handle PointMemorySCM::get_first_time(Handle map_name,
                                      Handle ato, Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);

	time_pt tp;
	bool r = tsa[map_name]->get_oldest_time_elapse_atom_observed(ato, tpt, tp);
	if (not r) return Handle(); // XXX should this throw instead?

	// Make and return atTimeLink
	return map_name->getAtomSpace()->add_atom(timestamp_tag_atom(tp, ato));
}

Handle PointMemorySCM::get_last_time(Handle map_name,
                                     Handle ato, Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);

	time_pt tp;
	bool r = tsa[map_name]->get_last_time_elapse_atom_observed(ato, tpt, tp);
	if (not r)
		return Handle::UNDEFINED; // XXX should this throw instead?

	// make and return atTimeLink
	return map_name->getAtomSpace()->add_atom(timestamp_tag_atom(tp, ato));
}

Handle PointMemorySCM::get_at_loc_ato(Handle map_name,
                             Handle loc)
{
	have_map(map_name);
	// loc should be a ListLink of three NumberNodes.
	const HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();
	return tsa[map_name]->get_atom_at_location(point3d(x, y, z));
}

Handle PointMemorySCM::get_past_loc_ato(Handle map_name,
                                        Handle loc,
                                        Handle elapse)
{
	// loc should be a ListLink of three NumberNodes.
	const HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();

	time_pt tpt = get_map_time(map_name, elapse);
	return tsa[map_name]->get_atom_at_time_by_location(tpt, point3d(x, y, z));
}

// Tag an atom with SetLink of locations (AtLocationLink)
// Returns an empty SetLink if the pointlist is empty.
static Handle tag_atom_with_locs(const Handle& map_name,
                                 const Handle& ato,
                                 const point3d_list& pl)
{
	HandleSeq loc_links;
	for (const point3d& pt: pl)
	{
		loc_links.push_back(Handle(createLink(AT_LOCATION_LINK,
			map_name,
			Handle(createLink(LIST_LINK, ato,
				Handle(createLink(LIST_LINK,
					Handle(createNumberNode(pt.x())),
					Handle(createNumberNode(pt.y())),
					Handle(createNumberNode(pt.z()))
				))
			))
		)));
	}
	return Handle(createLink(loc_links, SET_LINK));
}

Handle PointMemorySCM::get_first_location(Handle map_name,
                                          Handle ato, Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);
	point3d_list pl = tsa[map_name]->get_oldest_locations(ato, tpt);
	return map_name->getAtomSpace()->add_atom(tag_atom_with_locs(map_name, ato, pl));
}

Handle PointMemorySCM::get_last_location(Handle map_name,
                                         Handle ato, Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);
	point3d_list pl = tsa[map_name]->get_newest_locations(ato, tpt);
	return map_name->getAtomSpace()->add_atom(tag_atom_with_locs(map_name, ato, pl));
}

Handle PointMemorySCM::get_locs_ato(Handle map_name, Handle ato)
{
	have_map(map_name);
	point3d_list pl = tsa[map_name]->get_locations_of_atom(ato);
	return map_name->getAtomSpace()->add_atom(tag_atom_with_locs(map_name, ato, pl));
}

Handle PointMemorySCM::get_past_locs_ato(Handle map_name,
                                         Handle ato, Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);
	point3d_list pl = tsa[map_name]->get_locations_of_atom_at_time(tpt, ato);
	return map_name->getAtomSpace()->add_atom(tag_atom_with_locs(map_name, ato, pl));
}

Handle PointMemorySCM::get_elapse_list_at_loc_ato(Handle map_name,
                                        Handle ato,
                                        Handle loc)
{
	have_map(map_name);
	// loc should be a ListLink of three NumberNodes.
	const HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();

	time_list tl = tsa[map_name]->get_times_of_atom_occurence_at_location(point3d(x, y, z), ato);

	HandleSeq LL;
	for (const time_pt& tp: tl)
		LL.push_back(timestamp_tag_atom(tp, ato));

	return map_name->getAtomSpace()->add_atom(opencog::Handle(createLink(LL, SET_LINK)));
}

// Get the timeline of the atom.  That is, get a sequence of
// time-points, at which the atom is in the map.
Handle PointMemorySCM::get_timeline(Handle map_name, Handle ato)
{
	have_map(map_name);
	time_list tl = tsa[map_name]->get_timeline(ato);
	HandleSeq LL;
	for (const time_pt& tp: tl)
		LL.push_back(timestamp_tag_atom(tp, ato));
	return map_name->getAtomSpace()->add_atom(opencog::Handle(createLink(LL, SET_LINK)));
}

Handle PointMemorySCM::remove_location_ato(Handle map_name,
                                           Handle loc)
{
	have_map(map_name);
	// loc should be a ListLink of three NumberNodes.
	const HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();

	tsa[map_name]->remove_atoms_at_location(point3d(x, y, z));
	return loc;
}

Handle PointMemorySCM::remove_past_location_ato(Handle map_name,
                                        Handle loc,
                                        Handle elapse)
{
	// loc should be a ListLink of three NumberNodes.
	const HandleSeq& hs = loc->getOutgoingSet();
	double x = NumberNodeCast(hs[0])->get_value();
	double y = NumberNodeCast(hs[1])->get_value();
	double z = NumberNodeCast(hs[2])->get_value();

	time_pt tpt = get_map_time(map_name, elapse);
	tsa[map_name]->remove_atom_at_time_by_location(tpt, point3d(x, y, z));
	return loc;
}

Handle PointMemorySCM::remove_curr_ato(Handle map_name, Handle ato)
{
	have_map(map_name);
	tsa[map_name]->remove_atom_at_current_time(ato);
	return ato;
}

Handle PointMemorySCM::remove_past_ato(Handle map_name, Handle ato, Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);
	tsa[map_name]->remove_atom_at_time(tpt, ato);
	return ato;
}

Handle PointMemorySCM::remove_all_ato(Handle map_name, Handle ato)
{
	have_map(map_name);
	tsa[map_name]->remove_atom(ato);
	return ato;
}


time_pt PointMemorySCM::get_map_time(const Handle& map_name,
                                     const Handle& helapse)
{
	have_map(map_name);
	int elapse = get_int(helapse, "elapsed time (millisecs)");
	return tsa[map_name]->get_current_time() - std::chrono::milliseconds(elapse);
}

// Spatial query api assuming 1 atom in 1 map at 1 location.
// elapse is in milliseconds (in the past)
// For multi-map support, need to add features to main API, to provide
// more raw data results -ve for unknown
Handle
PointMemorySCM::get_distance_between(Handle map_name,
                                     Handle list,
                                     Handle elapse)
{
	const HandleSeq& hs = list->getOutgoingSet();
	time_pt tpt = get_map_time(map_name, elapse);
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(
		tsa[map_name]->get_distance_between(tpt, hs[0], hs[1]))));
}

// 2 = far, 1 = near, 0 = touching
Handle
PointMemorySCM::get_angular_nearness(Handle map_name,
                                     Handle list,
                                     Handle elapse)
{
	const HandleSeq& hs = list->getOutgoingSet();
	time_pt tpt = get_map_time(map_name, elapse);
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(
		tsa[map_name]->get_angular_nearness(tpt,
			hs[0], hs[1], hs[2]))));
}

// XXX FIXME this should just return xyz, and the left-right
// computation done elsewhere.
Handle
PointMemorySCM::get_target_is_right_left(Handle map_name,
                                         Handle list,
                                         Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);
	const HandleSeq& hs = list->getOutgoingSet();
	point3d res = tsa[map_name]->get_spatial_relations(tpt,
		hs[0], hs[1], hs[2]);
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(res.y())));
}

// XXX FIXME this should just return xyz, and the above-below
// computation done elsewhere.
Handle
PointMemorySCM::get_target_is_above_below(Handle map_name,
                                         Handle list,
                                         Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);
	const HandleSeq& hs = list->getOutgoingSet();
	point3d res = tsa[map_name]->get_spatial_relations(tpt,
		hs[0], hs[1], hs[2]);
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(res.z())));
}

// XXX FIXME this should just return xyz, and the front-back
// computation done elsewhere.
Handle
PointMemorySCM::get_target_is_front_back(Handle map_name,
                                         Handle list,
                                         Handle elapse)
{
	time_pt tpt = get_map_time(map_name, elapse);
	const HandleSeq& hs = list->getOutgoingSet();
	point3d res = tsa[map_name]->get_spatial_relations(tpt,
		hs[0], hs[1], hs[2]);
	return map_name->getAtomSpace()->add_atom(Handle(createNumberNode(res.x())));
}

void opencog_ato_pointmem_init(void)
{
	static PointMemorySCM pm;
}
