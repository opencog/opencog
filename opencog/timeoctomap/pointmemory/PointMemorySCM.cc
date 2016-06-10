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

#include <PointMemorySCM.h>
#include <opencog/guile/SchemePrimitive.h>

using namespace opencog::ato;
using namespace opencog;

/**
 * The constructor for PointMemorySCM.
 */
PointMemorySCM::PointMemorySCM()
{
//allocate point ato data structure object
    static bool is_init = false;
    if (is_init) return;
    space_res.push_back(SPACE_RES_M);
    //tsa = new TimeSpaceAtom(MEM_SEC*FPS,space_res);
    is_init = true;
    scm_with_guile(init_in_guile, this);
}
PointMemorySCM::~PointMemorySCM()
{
    //delete tsa;
    for (auto& x: tsa) {delete x.second;}
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
    define_scheme_primitive("create-map", &PointMemorySCM::create_map, this, "ato pointmem");
    define_scheme_primitive("get-time-res", &PointMemorySCM::get_time_res, this, "ato pointmem");
    define_scheme_primitive("get-space-res", &PointMemorySCM::get_space_res, this, "ato pointmem");
    define_scheme_primitive("get-time-units", &PointMemorySCM::get_time_units, this, "ato pointmem");
    define_scheme_primitive("map-ato", &PointMemorySCM::map_ato, this, "ato pointmem");
    define_scheme_primitive("get-first-ato", &PointMemorySCM::get_first_ato, this, "ato pointmem");
    define_scheme_primitive("get-last-ato", &PointMemorySCM::get_last_ato, this, "ato pointmem");
    define_scheme_primitive("get-at-loc-ato", &PointMemorySCM::get_at_loc_ato, this, "ato pointmem");
    define_scheme_primitive("get-past-loc-ato", &PointMemorySCM::get_past_loc_ato, this, "ato pointmem");
    define_scheme_primitive("get-locs-ato", &PointMemorySCM::get_locs_ato, this, "ato pointmem");
    define_scheme_primitive("get-past-locs-ato", &PointMemorySCM::get_past_locs_ato, this, "ato pointmem");
    define_scheme_primitive("get-elapse-list-at-loc-ato", &PointMemorySCM::get_elapse_list_at_loc_ato, this, "ato pointmem");
    define_scheme_primitive("get-elapse-list-ato", &PointMemorySCM::get_elapse_list_ato, this, "ato pointmem");
    define_scheme_primitive("remove-location-ato", &PointMemorySCM::remove_location_ato, this, "ato pointmem");
    define_scheme_primitive("remove-past-location-ato", &PointMemorySCM::remove_past_location_ato, this, "ato pointmem");
    define_scheme_primitive("remove-curr-ato", &PointMemorySCM::remove_curr_ato, this, "ato pointmem");
    define_scheme_primitive("remove-past-ato", &PointMemorySCM::remove_past_ato, this, "ato pointmem");
    define_scheme_primitive("remove_all_ato", &PointMemorySCM::remove_all_ato, this, "ato pointmem");
#endif
}

bool PointMemorySCM::create_map(string name,
                    double space_res_mtr,
                    int time_res_milli_sec,
                    int time_units)
{
//reject if name already exists
if (name.length()<1)return false;
//reject if time units < 1
if (time_units<1)return false;
//reject if time res,space_res <=0
if ((time_res<=0)||(space_res_mtr)<=0.0) return false;

std::map<string,TimeOctomap*>::iterator it;
it=tsa.find(name);
if (it != tsa.end()) return false;
tsa[name]=new TimeOctomap(time_units,space_res_mtr,std::chrono::milliseconds(time_res_milli_sec));
return true;
}
//add point clouds later
int PointMemorySCM::get_time_res(string map_name)
{
  duration_c dr=tsa[map_name]->get_time_resolution();
  return chrono::milliseconds(dr).count();
}
double PointMemorySCM::get_space_res(string map_name)
{
  return tsa[map_name]->get_space_resolution();
}
int PointMemorySCM::get_time_units(string map_name)
{
  return tsa[map_name]->get_time_units();
}

void PointMemorySCM::step_time_unit(string map_name)
{
  tsa[map_name]->step_time_unit();
}
bool PointMemorySCM::map_ato(string map_name,Handle ato,double x,double y,double z)
{
  return tsa[map_name]->put_atom_at_current_time(point3d(x,y,z),ato);
}
Handle PointMemorySCM::get_first_ato(string map_name,Handle ato,int elapse)
{
  time_pt tpt,result; duration_c dr;
  tsa[map_name]->get_current_time_range(tpt,dr);
  dr=std::chrono::milliseconds(elapse);
  tpt-=dr;
  bool r=tsa[map_name]->get_oldest_time_elapse_atom_observed(ato,tpt,result);
  if (!r) return UndefinedHandle;
  //make and return atTimeLink
  return;
}
Handle PointMemorySCM::get_last_ato(string map_name,Handle ato,int elapse)
{
  time_pt tpt,result; duration_c dr;
  tsa[map_name]->get_current_time_range(tpt,dr);
  dr=std::chrono::milliseconds(elapse);
  tpt-=dr;
  bool r=tsa[map_name]->get_last_time_elapse_atom_observed(ato,tpt,result);
  if (!r) return UndefinedHandle;
  //make and return atTimeLink
  return;

}
Handle PointMemorySCM::get_at_loc_ato(string map_name,double x,double y,double z)
{
  Handle ato;
  if (tsa[map_name]->get_atom_current_time_at_location(point3d(x,y,z),ato))
     return ato;
  return UndefinedHandle;
}
Handle PointMemorySCM::get_past_loc_ato(string map_name,int elapse,
                            double x,double y,double z)
{
  time_pt tpt; duration_c dr;
  tsa[map_name]->get_current_time_range(tpt,dr);
  dr=std::chrono::milliseconds(elapse);
  tpt-=dr;
  Handle ato;
  if (tsa[map_name]->get_atom_at_time_by_location(tpt,point3d(x,y,z),ato))
     return ato;
  return UndefinedHandle;

}
Handle PointMemorySCM::get_locs_ato(string map_name,Handle ato)//listlink atLocationLink
{
  point3d_list pl=tsa[map_name]->get_locations_of_atom_occurence_now(ato);
  if (pl.size()<1)return UndefinedHandle;
  HandleSeq loc_links(pl.size());
  while (pl.size()>0)
  {
    point3d pt=pl.front();
    loc_links.add(createLink(AT_LOCATION_LINK,
      createNode(OBJECT_NODE,map_name),
      createLink(LIST_LINK,ato,
        createLink(LIST_LINK,
          createNode(NUMBER_NODE,to_string(pt.x())),
          createNode(NUMBER_NODE,to_string(pt.y())),
          createNode(NUMBER_NODE,to_string(pt.z()))
        )
      )
    ));
    pl.pop_front();
  }//while
  
  return createLink(LIST_LINK,loc_links);
}
Handle PointMemorySCM::get_past_locs_ato(string map_name,Handle ato,int elapse)
{
  time_pt tpt; duration_c dr;
  tsa[map_name]->get_current_time_range(tpt,dr);
  dr=std::chrono::milliseconds(elapse);
  tpt-=dr;
  point3d_list pl=tsa[map_name]->get_locations_of_atom_occurence_at_time(tpt,ato);
  if (pl.size()<1)return UndefinedHandle;

  HandleSeq loc_links(pl.size());
  while (pl.size()>0)
  {
    point3d pt=pl.front();
    loc_links.add(createLink(AT_LOCATION_LINK,
      createNode(OBJECT_NODE,map_name),
      createLink(LIST_LINK,ato,
        createLink(LIST_LINK,
          createNode(NUMBER_NODE,to_string(pt.x())),
          createNode(NUMBER_NODE,to_string(pt.y())),
          createNode(NUMBER_NODE,to_string(pt.z()))
        )
      )
    ));
    pl.pop_front();
  }//while

  return createLink(LIST_LINK,loc_links);
}

Handle PointMemorySCM::get_elapse_list_at_loc_ato(string map_name,Handle ato,
              double x,double y,double z)//listlink atTimeLink
{
  time_list tl=tsa[map_name]->get_times_of_atom_occurence_at_location(point3d(x,y,z),ato);
  if (tl.size()<1)return UndefinedHandle;
  HandleSeq LL;
  while (tl.size()>0)
  {
    time_pt tp=tl.front();
    tl.pop_front();
    std::time_t ttp=std::chrono::system_clock::to_time_t(tp);
    string ts=std::put_time(std::localtime(&ttp), "%F %T ");
    //time_since_epoch gives duration, duration to seconds and milliseconds then subtract [millisec - sec to millisec] 
    //add milli sec to ts    
    LL.add(createLink(AT_TIME_LINK,createNode(TIME_NODE,ts),ato));
  }
  return LL;
}
Handle PointMemorySCM::get_elapse_list_ato(string map_name,Handle ato)//listlink atTimeLink
{
  time_list tl=tsa[map_name]->get_times_of_atom_occurence_in_map(ato);
  if (tl.size()<1)return UndefinedHandle;
  HandleSeq LL;
  while (tl.size()>0)
  {
    time_pt tp=tl.front();
    tl.pop_front();
    LL.add(createLink(AT_TIME_LINK,createNode(TIME_NODE,),ato));
  }
  return LL;
}
bool PointMemorySCM::remove_location_ato(string map_name,double x,double y,double z)
{
  return tsa[map_name]->remove_atom_at_current_time_by_location(point3d(x,y,z));
}
bool PointMemorySCM::remove_past_location_ato(string map_name,int elapse,
         double x,double y,double z)
{
  time_pt tpt; duration_c dr;
  tsa[map_name]->get_current_time_range(tpt,dr);
  dr=std::chrono::milliseconds(elapse);
  tpt-=dr;
  return tsa[map_name]->remove_atom_at_time_by_location(tpt,point3d(x,y,z));
}
void PointMemorySCM::remove_curr_ato(string map_name,Handle ato)
{
  tsa[map_name]->remove_atom_at_current_time(ato);
}
void PointMemorySCM::remove_past_ato(string map_name,Handle,int elapse)
{
  time_pt tpt; duration_c dr;
  tsa[map_name]->get_current_time_range(tpt,dr);
  dr=std::chrono::milliseconds(elapse);
  tpt-=dr;
  tsa[map_name]->remove_atom_at_time(tpt,ato);
}
void PointMemorySCM::remove_all_ato(string map_name,Handle ato)
{
  tsa[map_name]->remove_atom(ato);
}

void opencog_ato_pointmem_init(void)
{
    static PointMemorySCM pm;
}
