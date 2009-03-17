#ifndef COMPAREATOMTREETEMPLATE_H
#define COMPAREATOMTREETEMPLATE_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/util/tree.h>

using namespace opencog;

typedef tree<Vertex> atom_tree;
typedef atom_tree::iterator atom_tree_it;

//Methods to compare a atom_tree template and atom_tree

/**
 * That relation is not an equivalence operator but a parial order
 * that is argument tr1 is used a template and tr2 is being checked is an
 * instantiation of temple tr1
 */
struct is_atom_tree_template_of : public std::binary_function<atom_tree,
                                                              atom_tree,
                                                              bool> {
  bool operator()(const atom_tree& tr1, const atom_tree& tr2,
		  const AtomSpace* as, bool ec) const;

private:
	bool is_atom_tree_template_it(const atom_tree& tr1, atom_tree_it it1,
				const atom_tree& tr2, atom_tree_it it2,
				const AtomSpace* as, bool ec) const;

};


/**
 * check is a given Handle corresponds to an atom structured like
 * a given template
 *
 * It allows to take or not into account the number
 * of children
 */
struct does_fit_template {
	const atom_tree& _tr;
	const opencog::AtomSpace* _as;
	bool _enable_childcount;
			   /*
			    * this allows to ignore or not the number
			    * of children of all vertices
			    * in the comparison
			    * that is if _enable_childcount is true
			    * then EvalLink(NODE) != EvalLink("name":NODE, other nodes...)
			    * while if _enable_childcount is false
			    * this is going to be equal
			    */

	does_fit_template(const atom_tree& tr, const opencog::AtomSpace* as, bool ec = false);

	bool operator()(opencog::Handle h) const;

private:

	void expandHandletree(bool fullVirtual, atom_tree& ret, atom_tree_it ret_top,
							const opencog::AtomSpace* as) const;

	//take a Handle and build the atom_tree corresponding to it
	//produces a tree with Handle of type Link at its vertices and Handle of type
	//Node at its leaves.
	//fullvirtual is true then Handles are replaced by their types
	//(Handle seen as Type).
	atom_tree make_atom_tree(opencog::Handle h, const opencog::AtomSpace* as, bool fullVirtual) const;

	void makeHandletree(opencog::Handle real, bool fullVirtual, atom_tree& ret,
						const opencog::AtomSpace* as) const;
};

#endif
