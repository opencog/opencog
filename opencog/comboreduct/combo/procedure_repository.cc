/*
 * opencog/comboreduct/combo/procedure_repository.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#include "procedure_repository.h"
#include "../type_checker/type_tree.h"
#include "iostream_combo.h"

#include <boost/version.hpp>

// There is some kind of bug in Boost version 1.40 through 1.44
#if ((BOOST_VERSION >= 104000) && (BOOST_VERSION <= 104400))
#define IGNORE_BOOST_GRAPH
#endif

#ifndef IGNORE_BOOST_GRAPH
#include <boost/graph/strong_components.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/topological_sort.hpp>
#endif

#include <cstdio>

namespace opencog { namespace combo {

typedef combo_tree::iterator pre_it;
typedef combo_tree::sibling_iterator sib_it;

void procedure_repository::add(procedure_call_base* pc) {
    std::string name = pc->get_name();
    str_proc_map_const_it spmi = _repo.find(name);
    if(spmi==_repo.end())
        _repo[name]=pc;
    else {
        //if already exists then check if there are equal in content,
        //if so then do not add it
        //but if not equal in content then raise a cassert
        OC_ASSERT(*(spmi->second)==*pc, "A procedure named %s with a different body function has already been declared, you cannot overwrite it you must first explicitely remove it using the remove method", name.c_str());
    }
}

void procedure_repository::remove(const std::string& name) {
    _repo.erase(name);
}

procedure_call procedure_repository::instance(const std::string& name) const {
    str_proc_map_const_it spmi = _repo.find(name);
    return spmi==_repo.end() ? NULL : spmi->second;    
}

bool procedure_repository::does_contain(const std::string& name) const {
    return _repo.find(name)!=_repo.end();
}

void procedure_repository::instantiate_procedure_calls(bool wodo) {
    for(str_proc_map_it ip = _repo.begin(); ip != _repo.end(); ++ip)
        instantiate_procedure_calls(ip->second->get_mutable_body(), wodo);
}

void procedure_repository::instantiate_procedure_calls(combo_tree& tr,
                                                       bool wodo) const {
    for(pre_it it = tr.begin(); it != tr.end(); ++it) {
        // is_definite_object is used here because it corresponds to a
        // string which is where procedure names are typed before
        // resolution has appended.
        if(is_definite_object(*it)) {
            std::string str = get_definite_object(*it);
            str_proc_map_const_it proc=_repo.find(str);
            if(proc==_repo.end()) {
                if(wodo) {
                    opencog::logger().debug("ComboProcedureRepository - Creating definite_object '%s'.", str.c_str());
                }
            } else {
                opencog::logger().debug("ComboProcedureRepository - Replacing procedure name by procedure object '%s'", str.c_str());
                *it=proc->second;
            }
        }
    }
}

std::set<const procedure_call_base*> procedure_repository::procedure_call_dependencies(const procedure_call_base* pc) const {
    std::set<const procedure_call_base*> res;
    OC_ASSERT(pc, "Not that must be an assumption, could return empty set as well");
    const combo_tree& body = pc->get_body();
    for(combo_tree::iterator it = body.begin(); it != body.end(); ++it)
        if(is_procedure_call(*it))
            res.insert(get_procedure_call(*it));
    return res;
}

std::set<const procedure_call_base*> procedure_repository::procedure_call_dependencies(const std::set<const procedure_call_base*>& pcs) const {
    procedure_call_set tmp, res;
    for(procedure_call_set_const_it spi = pcs.begin();
        spi != pcs.end(); ++spi) {
        procedure_call_set d = procedure_call_dependencies(*spi);
        set_union(d.begin(), d.end(), tmp.begin(), tmp.end(),
                  std::insert_iterator<procedure_call_set>(tmp, tmp.begin()));
    }
    set_difference(tmp.begin(), tmp.end(), pcs.begin(), pcs.end(),
                   std::insert_iterator<procedure_call_set>(res, res.begin()));
    return res;
}

void procedure_repository::generate_and_order_strongly_connected_components() {
#ifndef IGNORE_BOOST_GRAPH
    //pc_vec is a vector containing all procedure calls of the repository
    //pv_index_map map all procedure calls to that vector indices
    std::vector<procedure_call> pc_vec;
    std::map<procedure_call, unsigned int> pc_index_map;
    for(str_proc_map_const_it spmi = _repo.begin(); spmi != _repo.end();
        ++spmi) {
        pc_index_map[spmi->second] = pc_vec.size();
        pc_vec.push_back(spmi->second);

        //debug print
        //std::cout << "INDEX : " << pc_vec.size()-1 << " PC : " << spmi->second
        //	<< std::endl;
        //~debug print

    }
    //define and fill the dependency graph
    typedef boost::adjacency_list<> Graph;
    Graph g(pc_vec.size());
    for(unsigned int i = 0; i < pc_vec.size(); ++i) {
        std::set<procedure_call> spc = procedure_call_dependencies(pc_vec[i]);
        for(std::set<procedure_call>::const_iterator spci = spc.begin();
            spci != spc.end(); ++spci) {
            add_edge(pc_index_map[*spci], i, g);
        }
    }
    //compute the strongly connected components
    typedef boost::graph_traits<Graph>::vertex_descriptor GVertex;

    std::vector<unsigned int> component(num_vertices(g));
    std::vector<unsigned int> discover_time(num_vertices(g));
    std::vector<boost::default_color_type> color(num_vertices(g));
    std::vector<GVertex> root(num_vertices(g));
    int num = strong_components(g, &component[0], 
                                boost::root_map(&root[0]).
                                color_map(&color[0]).
                                discover_time_map(&discover_time[0]));
    
    //print debug
    //std::cout << "PARTITION SIZE : " << num << std::endl;
    //for(unsigned int i = 0; i < component.size(); ++i) {
    //  std::cout << "INDEX : " << i << " CORRESPONDING TO SCHEMA : "
    //	<< pc_vec[i] << " IS IN CATEGORY : " << component[i]
    //	<< std::endl;
    //}
    //~print debug

    //create the abstract dependency graph where nodes are sets of strongly
    //connected components
    strongly_connected_components scc(num);
    //create vertices
    for(unsigned int i = 0; i < component.size(); ++i) {
        scc[component[i]].insert(pc_vec[i]);
    }
    //print debug
    //for(unsigned int i = 0; i < scc.size(); ++i) {
    //  std::cout << "SCC INDEX : " << i << " SET :" ;
    //  for(procedure_call_const_it pci = scc[i].begin();
    //  pci != scc[i].end(); ++pci) {
    //std::cout << " " << *pci;
    //}
    //std::cout << std::endl;
    //}
    //~print debug
    Graph hg(num); //the partition graph
    //create edges
    for(unsigned int i = 0; i < scc.size(); ++i) {
        procedure_call_set pcs = procedure_call_dependencies(scc[i]);
      
        //print debug
        //std::cout << "SCC INDEX : " << i << " SET :" ;
        //for(procedure_call_const_it pci = scc[i].begin();
        //  pci != scc[i].end(); ++pci) {
        //std::cout << " " << *pci;
        //}
        //std::cout << " DEPENDS ON :";
        //for(procedure_call_const_it pci = pcs.begin(); pci != pcs.end(); ++pci) {
        //	std::cout << " " << *pci;
        //}
        //std::cout << std::endl;
        //~print debug

        //get the set of all components that contain procedure_call of pcs
        std::set<unsigned int> outgoing_scc;
        for(procedure_call_set_const_it pci = pcs.begin();
            pci != pcs.end(); ++pci) {
            outgoing_scc.insert(component[pc_index_map[*pci]]);
        }
        //add all edges pointing to outgoing_scc
        for(std::set<unsigned int>::iterator sui = outgoing_scc.begin();
            sui != outgoing_scc.end(); ++sui) {
            add_edge(*sui, i, hg);
            //print debug
            //std::cout << "EDGE : " << *sui << " " << i << std::endl;
            //~print debug
        }
    }

    //compute topological order of hg
    std::vector<unsigned int> ordered_hvertices;
    topological_sort(hg, std::back_inserter(ordered_hvertices));

    //affect the result in _ordered_scc
    //reverse iterator is used to place first more independent procedures
    _ordered_scc.clear();
    for(std::vector<unsigned int>::reverse_iterator 
            vui = ordered_hvertices.rbegin();
        vui != ordered_hvertices.rend(); ++vui) {
        procedure_call_set& pcs = scc[*vui];
        _ordered_scc.push_back(pcs);
    }
    //debug print
    //for(strongly_connected_components_const_it sci = _ordered_scc.begin();
    //sci != _ordered_scc.end(); ++sci) {
    //const procedure_call_set& pcs = *sci;
    //std::cout << "ORDERED_SCC :";
    //for(procedure_call_set_const_it pci = pcs.begin();
    //  pci != pcs.end(); ++pci) {
    //std::cout << " " << *pci;
    //}
    //std::cout << std::endl;
    //}
    //~debug print
#endif
}

bool procedure_repository::infer_types_repo() {
    //proceed following the dependency order, less dependent first
    //for each class of strongly connected components
    //for each procedure of tha class infer
    //1) the contextual type_tree (that is the apparent type of the procedure
    //deduced by the way it is used in the body procedure).
    //The result is stored in proc_type structure
    //2) the type of each argument of the procedure, stored in the structure
    //proc_arg_type.
    //
    //For each stronly connected component class
    //the algorithm iterate over those 2 maps as long as they change.
    //When there is no more change it checks and infers the types 
    //of all procedures of the class based on body content rather than context
    //(using get_type_tree and reducing it)
    //and checks that there are compatible with the previously infered types

    typedef std::map<procedure_call, type_tree> proc_type;
    typedef proc_type::iterator proc_type_it;
    typedef std::map<procedure_call, type_tree_seq> proc_arg_type;

    generate_and_order_strongly_connected_components();

    bool all_went_well = true;

    for(strongly_connected_components_it sci = _ordered_scc.begin();
        sci != _ordered_scc.end(); ++sci) {
        proc_type pt, pt_tmp;
        proc_arg_type pat, pat_tmp;
        //perform contextual inference first
        do {
            pt_tmp = pt;
            pat_tmp = pat;
            for(procedure_call_set_it pci = sci->begin();
                pci != sci->end(); ++pci) {
                procedure_call current_pc = *pci;

                //insert argument_list to pat[current_pc]
                pat[current_pc] = type_tree_seq();
                type_tree_seq& atl = pat.find(current_pc)->second;
                //insert unknown type for current_pc in pt
                pt[current_pc] = type_tree(id::unknown_type);

                //look over the body of the procedure to infer procedure and argument
                //types
                combo_tree body = current_pc->get_body();
                for(pre_it it = body.begin(); it != body.end(); ++it) {
                    vertex v = *it;
                    //infer procedure types, compare
                    if(is_procedure_call(v)) {
                        procedure_call_base* pc =
                            const_cast<procedure_call_base*>(get_procedure_call(v));

                        type_tree infered_type = infer_vertex_type(body, it, atl);

                        if(!is_well_formed(infered_type)) {
                            std::stringstream iss;
                            std::stringstream tss;
                            std::stringstream rss;
                            std::stringstream pss;
                            iss << *it;
                            rss << combo_tree(it);
                            tss << infered_type;
                            pre_it par = body.parent(it);
                            bool valid_par = body.is_valid(par);
                            if(valid_par)
                                pss << *par;
                            std::string error_message = std::string("The procedure '")
                                + iss.str()
                                + std::string("' (root of the subtree ")
                                + rss.str()
                                + (valid_par?std::string(" and child of ") + pss.str()
                                   :std::string(""))
                                + std::string(") seems not to fit with the procedure '")
                                + current_pc->get_name()
                                + std::string("', because the infered type of ")
                                + iss.str()
                                + std::string(" (which is ")
                                + tss.str()
                                + std::string(") is not well formed");
                            OC_ASSERT(false, error_message.c_str());
                        }

                        //print debug
                        //std::cout << "CONTEXTUAL INFER : " << (unsigned int)pc << " : ";
                        //pc->toStream(std::cout, true);
                        //std::cout << std::endl;
                        //std::cout << "RESULT IS : " << infered_type << std::endl;
                        //~print debug

                        //if pc belongs to the current strongly connected partition
                        //then add it to pt and possibly get its intersection with
                        //previously infer type of itself in other context
                        if(sci->find(pc)!=sci->end()) {
                            proc_type_it pt_it = pt.find(pc);
                            if(pt_it==pt.end()) {
                                pt[pc]=infered_type;
                                //before setting the new type tree check that the arity is
                                //the same as the old one.
                                //Indeed the arity should always be correct
                                //whether it is type checked in context or in content
                                //and if it has never been type checked before
                                //it should be given when the procedure is loaded from file
                                arity_t pa = pc->arity();
                                arity_t ia = type_tree_arity(infered_type);
                                if(pa!=ia)
                                    printf("The arity of %s has been contextually infered at %d which is different from the previously given or infered arity %d\n", pc->get_name().c_str(), ia, pa);
                                OC_ASSERT(pa==ia, "The arity of %s has been contextually infered at %d which is different from the previously given or infered arity %d", pc->get_name().c_str(), ia, pa);
                                pc->set_type_tree(infered_type);
                            }
                            else {
                                type_tree& tt = pt_it->second;
                                tt = get_intersection(tt, infered_type);
                                //before setting the new type tree check that the arity is
                                //the same as the old one.
                                //Indeed the arity should always be correct
                                //whether it is type checked in context or in content
                                //and if it has never been type checked before
                                //it should be given when the procedure is loaded from file
                                arity_t pa = pc->arity();
                                arity_t ia = type_tree_arity(tt);
                                if(pa!=ia)
                                    printf("The arity of %s has been contextually infered at %d which is different from the previously given or infered arity %d\n", pc->get_name().c_str(), ia, pa);
                                OC_ASSERT(ia==pa, "The arity of %s has been contextually infered at %d which is different from the previously given or infered arity %d", pc->get_name().c_str(), ia, pa);
                                pc->set_type_tree(tt);
                                if(!is_well_formed(tt)) {
                                    printf("Something is wrong in the way %s is used in %s\n",
                                           pc->get_name().c_str(),
                                           current_pc->get_name().c_str());
                                    OC_ASSERT(false,
                                              "Something is wrong in the way %s is used in %s",
                                              pc->get_name().c_str(),
                                              current_pc->get_name().c_str());
                                    all_went_well = false;
                                }
                            }
                        }
                        //otherwise pc does not belong to the partition and
                        //then just check if the previously non contextual infered one
                        //(supposely correct) inherits from it
                        else if(!inherit_type_tree(pc->get_type_tree(), infered_type)){
                            std::stringstream ssit, ssct;
                            ssit << infered_type;
                            ssct << pc->get_type_tree();
                            printf("Something must be wrong in the way %s is used in %s or in the definition of %s, because the type contextually infered of %s (which is %s) does not inherit its type based on its content (which is %s)\n",
                                   pc->get_name().c_str(),
                                   current_pc->get_name().c_str(),
                                   pc->get_name().c_str(), pc->get_name().c_str(),
                                   ssit.str().c_str(), ssct.str().c_str());
                            OC_ASSERT(false,
                                      "Something must be wrong in the way %s is used in %s or in the definition of %s, because the type contextually infered of %s (which is %s) does not inherit its type based on its content (which is %s)",
                                      pc->get_name().c_str(),
                                      current_pc->get_name().c_str(),
                                      pc->get_name().c_str(), pc->get_name().c_str(),
                                      ssit.str().c_str(), ssct.str().c_str());
                            all_went_well = false;
                        }
                    }
                    //infer argument type, compare
                    else if(is_argument(v)) {
                        argument a = get_argument(v);
                        type_tree infered_arg_type = infer_vertex_type(body, it, atl);
                        if(!is_well_formed(infered_arg_type)) {
                            std::stringstream iss;
                            std::stringstream tss;
                            std::stringstream pss;
                            iss << *it;
                            tss << infered_arg_type;
                            pre_it par = body.parent(it);
                            bool valid_par = body.is_valid(par);
                            if(valid_par)
                                pss << *par;
                            std::string error_message = std::string("The input argument '")
                                + iss.str()
                                + (valid_par?std::string("' (child of ")
                                   +pss.str()+std::string(")")
                                   :std::string("'"))
                                + std::string(" seems not to fit with the procedure '")
                                + current_pc->get_name()
                                + std::string("', because the infered type of '")
                                + iss.str()
                                + std::string("' (which is ")
                                + tss.str()
                                + std::string(") is not well formed");
                            OC_ASSERT(false, error_message.c_str());
                        }
                        type_tree att = get_arg_type(a, atl);
                        type_tree itt = get_intersection(att, infered_arg_type);
                        set_arg_type(itt, a, atl);
                    }
                }
            }
        } while(pt!=pt_tmp || pat!=pat_tmp);

        //infer the type tree of all procedures of the partition
        //using get_type_tree and reduce to a single tree,
        //then compare if the result is compatible with
        //(that is inherits from) the previously contextual inference

        for(procedure_call_set_it pci = sci->begin(); pci != sci->end(); ++pci) {
            procedure_call_base* pc = const_cast<procedure_call_base*>(*pci);

            //infer type of pc based on its body content
            type_tree tt = get_type_tree(pc->get_body());
      
            //debug print
            //std::cout << "TR NAME: " << pc->get_name() << std::endl;
            //std::cout << "TT: " << tt << std::endl;
            //std::cout << "TR: " << pc->get_body() << std::endl;
            //~debug print

            //debug print
            //std::cout << "BODY CONTENT INFER: " << (unsigned int)pc << " : ";
            //pc->toStream(std::cout, true);
            //std::cout << std::endl;
            //std::cout << "TT BEFORE REDUCTION: " << tt << std::endl;
            //~debug print

            reduce_type_tree(tt, pat[pc], pc->get_body(), pc->get_name());
            insert_arg_type_tree(pat[pc], tt);

            //debug print
            //std::cout << "RESULT IS : " << tt << std::endl;
            //~debug print

            //check that tt is compatible with previously contextual infered type
            //and if so affect it as type tree of pc
            if(inherit_type_tree(tt, pt[pc])) {
                //before setting the new type tree check that the arity is
                //the same as the old one.
                //Indeed the arity should always be correct
                //whether it is type checked in context or in content
                //and if it has never been type checked before
                //it should be given when the procedure is loaded from file
                arity_t pa = pc->arity();
                arity_t ia = type_tree_arity(tt);
                pc->set_type_tree(tt);
                OC_ASSERT(ia==pa,
                          "The arity of '%s' has been infered in content at %d which is different from the previously given or infered arity %d", pc->get_name().c_str(), ia, pa);
            }
            else {
                all_went_well = false;
                std::stringstream tt_ss, pt_ss, message;
                tt_ss << tt;
                pt_ss << pt[pc];
                message << "procedure_repository::infer_types_repo -"
                        << " Type check error in procedure '"
                        << pc->get_name()
                        << "'. The type infered from context, which is '"
                        << tt_ss.str()
                        << "', does not inherit the type previously known or infered"
                        << ", which is '"
                        << pt_ss.str()
                        << "'. Look at the log for more information on"
                        << " why it failed or above on the standard"
                        << " output if the"
                        << " log has been redirected to it.";
                OC_ASSERT(false, message.str().c_str());
            }
        }
    }
    return all_went_well;
}

void procedure_repository::clear() {
    _repo.clear();
}

std::ostream& procedure_repository::toStream(std::ostream& out, bool with_type) const {
    for(str_proc_map_const_it i = _repo.begin(); i != _repo.end(); ++i) {
        std::stringstream ss;
        i->second->toStream(ss, true);
        out << ss.str();
        if(with_type)
            out << " [" << i->second->get_type_tree() << "]";
        out << std::endl;
    }
    return out;
}

void procedure_repository::print(bool with_type) const {
    toStream(std::cout, with_type);
}

std::ostream& operator<<(std::ostream& out, procedure_repository pr) {
    return pr.toStream(out);
}

} // ~namespace combo
} // ~namespace opencog
