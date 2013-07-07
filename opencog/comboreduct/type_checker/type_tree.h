/*
 * opencog/comboreduct/combo/type_tree.h
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
#ifndef _COMBO_TYPE_TREE_H
#define _COMBO_TYPE_TREE_H

#include <exception>

#include <opencog/util/tree.h>
#include <opencog/util/numeric.h>
#include <opencog/util/exceptions.h>

#include "type_tree_def.h"
#include "../crutil/exception.h"
#include "../combo/vertex.h"
#include "../combo/using.h"
#include "../combo/common_def.h"
#include "../combo/perception.h"
#include "../combo/procedure_call.h"

namespace opencog { namespace combo {

// templates helpers to convert C++ type into type_node
/// @note if I don't inline I get linkage error, go figure...
template<typename T>
inline type_node type_node_of() { return id::ill_formed_type; }

template<>
inline type_node type_node_of<bool>() { return id::boolean_type; }
        
template<>
inline type_node type_node_of<contin_t>() { return id::contin_type; }

// returns the arity of builtin b
char get_arity(builtin b);

// returns the type tree of builtin b
type_tree get_type_tree(builtin b);

// returns the output type of builtin b
type_tree get_output_type_tree(builtin b);

// returns the type tree of i-th argument of action b
//if there is no argument at i it returns the empty type_tree
type_tree get_input_type_tree(builtin b, arity_t i);

// returns the type tree of action a
type_tree get_type_tree(action a);

// returns the output type of action a
type_tree get_output_type_tree(action a);

// returns the type tree of i-th argument of action a
//if there is no argument at i it returns the empty type_tree
type_tree get_input_type_tree(action a, arity_t i);

// returns the type tree of builtin_action a
type_tree get_type_tree(builtin_action a);

// returns the output type of builtin_action a
type_tree get_output_type_tree(builtin_action a);

//returns the type tree of i-th input argument of builtin_action a
//if there is no input argument at i it returns the empty type_tree
type_tree get_input_type_tree(builtin_action a, arity_t i);

// returns the type tree of a perception p
type_tree get_type_tree(perception p);

// returns the output type of a perception p
type_tree get_output_type_tree(perception p);

// returns the type tree of i-th input argument of a perception p
// if there is no input argument at i it returns the empty type_tree
type_tree get_input_type_tree(perception p, arity_t i);

// all other get_type_tree functions
type_tree get_type_tree(const argument& a);
type_tree get_type_tree(contin_t t);
type_tree get_type_tree(const definite_object& d);
type_tree get_type_tree(indefinite_object i);
type_tree get_type_tree(const message& m);
type_tree get_type_tree(action_symbol as);
type_tree get_type_tree(wild_card wc);

// returns the node type of the output of a vertex
type_tree get_output_type_tree(const vertex& v);

// returns the type tree of i-th input argument of a vertex
// if there is no input argument at i it returns the empty type_tree
type_tree get_input_type_tree(const vertex& v, arity_t i);

// return the type_tree of v
type_tree get_type_tree(const vertex& v);

// returns true iff ty1 inherits ty2 and ty2 inherits ty1
bool equal_type_tree(const type_tree& ty1, const type_tree& ty2);

struct equal_to_type_tree : std::unary_function<const type_tree&, bool> {
private:
    const type_tree& _tt;
public:
    equal_to_type_tree(const type_tree& tt) : _tt(tt) {}
    bool operator()(const type_tree& tt) {
        return equal_type_tree(_tt, tt);
    }
};

/**
 * take 2 type_tree and check that ty1 inherit from ty2
 * It means :
 * 1) on a set interpretation that the set of combos that
 * satisfies ty1 is included in the set of combos that satisfies ty2
 * (this depends on the definition of what set of terms
 * corresponds to a given type_tree which is defined below)
 * 2) for functions, functions that can be applied over an equal or bigger
 * set of terms and return the results to an equal or smaller set of
 * a given type, inherits that type. i.e. ->(T1 T2 T3) inherit ->(T1' T2' T3')
 * if T1' inherit from T1, T2' inherit from T2 and T3 inherit from T3'
 * 3) there is one weird execption which is when the type uses unknown_type
 * then ->(T1 T2) inherits ->(unknown_type T2), even though unknown_type
 * clearly does not inherit T1...
 *
 * So for each operator :
 * union_type(T1 T2) is the set union of the sets of terms typed T1 and T2
 * now the other type operator are more restricting to make the type system
 * fairly simple :
 * lambda_type(T1 T2 T3) is the set of terms that take in argument
 * exactly terms of T1 and T2 (for first and second arguments) and return
 * any subset of T3
 * so for instance if T3' inherits from T3,
 * then lambda_type(T1 T2 T3') inherits from lambda_type(T1 T2 T3)
 * but if T1' inherit from T1 then
 * lambda_type(T1' T2 T3) does not inherit from lambda_type(T1 T2 T3)
 * neither the other way, they are just different sets of terms
 * Finally arg_list(T) does not inherit from T, neither the other way around
 * they are just different set of terms
 * so for instance + is typed lambda_type(arg_list(contin) contin)
 * and +($1 $2 $3) is type lambda_type(contin contin contin contin)
 * but neither one or the other has any inheritance relationship
 * So the type system is extremly basic and limited but easy to implement
 * and probably enough for our current practical needs
 * Last important point, to work correctly the 2 types must be reduced
 * that is application_type(lambda_type... will not be considered
 * Also, although lambda_type(T) is equal to T (because a constant
 * is just a function without arguments) it will not be treated as well
 * because such type_tree would have been previously reduced
 */
bool inherit_type_tree(const type_tree& ty1, const type_tree& ty2);

struct inherit_from_type_tree : std::unary_function<const type_tree&, bool> {
private:
    const type_tree& _tt;
public:
    inherit_from_type_tree(const type_tree& tt) : _tt(tt) {}
    bool operator()(const type_tree& tt) {
        return inherit_type_tree(tt, _tt);
    }
};

// like above but consider it1 and it2 as root of the ty1 and ty2
bool inherit_type_tree(const type_tree& ty1, type_tree_pre_it it1,
                       const type_tree& ty2, type_tree_pre_it it2);

// That reduction function performs the following reductions on type_tree :
//
// 1) reduce chains of application and abstraction and so on.
//    Fairly limited, do not handle union between application and lambda
//    and consider that the application is always complete
//    that is if the function has 3 arguments then the application
//    provides 3 elements (with the expection of arg_list where the number of
//    arguments is arbitrary)
// 2) reduce lambda(X) -> X
//    that is a function with no argument is reducible to a constant
// 3) reduce application(X) -> X
//    that is a function with no applied arguments remains unchanged
// 4) reduce union(X Y) -> union(X) when Y inherit from X
// 5) replace a variable by its output type
//
// Note that tr and proc_name are only passed for debugging message
void reduce_type_tree(type_tree& tt,
                      const type_tree_seq& arg_types,
                      const combo_tree& tr = combo_tree(),
                      const std::string& proc_name = std::string("PROCEDURE NAME UNKNOWN"));

// like above but assums that there is no variable in the type_tree
// (or that all variables are of type unknown_type)
void reduce_type_tree(type_tree& tt,
                      const combo_tree& tr = combo_tree(),
                      const std::string& proc_name = std::string("PROCEDURE NAME UNKNOWN"));

//reduce chain of application of abstraction and so on
void reduce_type_tree(type_tree& tt, type_tree_pre_it it,
                      const type_tree_seq& arg_types,
                      const combo_tree& tr, combo_tree::iterator ct_it,
                      const std::string& proc_name);

// Return the intersection of 2 types. Formally the intersection of 2
// types T1 and T2 is defined by the highest type (in the inheritance
// hierarchy) that inherits both T1 and T2. For now only a simpler
// definition is implemented yet practical enough
//
// for example :
//
// intersection of union(T1, T2) and union(T1, T3) is T1
//
// intersection of unknown_type and T is T
//
// intersection of ill_formed and T is ill_formed
//
// TODO : lambda
//
// Of course the case if T1 inherit T2 then interection of T1 and T2
// is T1 is also implemented. If the interection is ill_formed or
// anything else then the type_tree returned is ill_formed
type_tree get_intersection(const type_tree& tt1, const type_tree& tt2);

type_tree get_intersection(const type_tree& tt1, type_tree_pre_it it1,
                           const type_tree& tt2, type_tree_pre_it it2);

// infer type of a vertex based on its contextual use.  Note that such
// it will not be able to infer function that takes arg_list in
// argument.
type_tree infer_vertex_type(const combo_tree& tr, combo_tree::iterator it,
                            const type_tree_seq& atl = empty_tts);

// return the set of absolute idx corresponding to the arguments of tr
arity_set get_argument_abs_idx_set(const combo_tree& tr);
// like above but idx start from 0
arity_set get_argument_abs_idx_from_zero_set(const combo_tree& tr);

// infer the type_tree of each argument of a given combo_tree
void infer_arg_type_tree(const combo_tree& tr, type_tree_seq& arg_types);

// insert the types of the arguments in arg_types in the signature of
// a given function type. If the function is a constant then wrap it
// in lambda_type
void insert_arg_type_tree(const type_tree_seq& arg_types,
                          type_tree& tt2);

// function to fill a vector of type_tree corresponding to a given
// arguments
void set_arg_type(const type_tree& tt, const argument& arg,
                  type_tree_seq& arg_types);
// like above but takes a type_node corresponding to an argument
// it is assumed that the given type_node does correspond to an argument
void set_arg_type(const type_tree& tt, type_node arg,
                  type_tree_seq& arg_types);
// like above but uses the index of the argument
// (counted from 1, like in class argument of vertex.h)
void set_arg_type(const type_tree& tt, unsigned int idx,
                  type_tree_seq& arg_types);
// return the type_tree stored in the vector arg_types
// if no type_tree are stored then return the type_tree id::unknown_type
const type_tree& get_arg_type(const argument& arg,
                              const type_tree_seq& arg_types);
// like above but takes a type_node corresponding to an argument
// it is assumed that the given type_node does correspond to an argument
const type_tree& get_arg_type(type_node arg,
                              const type_tree_seq& arg_types);
// like above but uses the index of the argument
// (counted from 1, like in class argument of vertex.h)
const type_tree& get_arg_type(unsigned int idx,
                              const type_tree_seq& arg_types);

/// Generate type tree from a combo_tree.
/// does not perform any type checking or reduction.
type_tree get_type_tree(const combo_tree& tr);

// like above but consider it as root of tr
type_tree get_type_tree(const combo_tree& tr, combo_tree::iterator it);

/// infer_type_tree generate the type signature (i.e. in reduced form)
/// of a given combo_tree.
/// If a type error is detected then the type_tree will contain
/// ill_formed_type
type_tree infer_type_tree(const combo_tree& tr);

// ----------------------------------------------------------
// Arity-related functions

/// Return the number of input arguments of type tree.
///
/// Given the input :
///    ->(T_1 ... T_n T)
/// if T_n != arg_list(T')
///    then returns n
/// else (i.e. T_n == arg_list(T'))
///   then return -n
///
/// And finaly if it's not a function then returns 0
/// Note that it is assumed that if there is arg_list in the arguments
/// then it solely one and it is the last argument
arity_t type_tree_arity(const type_tree& ty);

/// Return the number of arguments of type contin.
///
/// If the input is of the form ->(T_1 ... T_n T) then this function
/// returns the number of T_k that are contins; else it returns zero.
arity_t contin_arity(const type_tree& ty);

/// Return the number of arguments of type boolean.
///
/// If the input is of the form ->(T_1 ... T_n T) then this function
/// returns the number of T_k that are boolean; else it returns zero.
arity_t boolean_arity(const type_tree& ty);

// WARNING : this should be defined in action.h but could not do that
// due  to dependency issues
//
/// Return the number of arguments of type action_result
///
/// If the input is of the form ->(T_1 ... T_n T) then this function
/// returns the number of T_k that are of type action_result; else it
/// returns zero.
arity_t action_result_arity(const type_tree& ty);

// ----------------------------------------------------------

/// Return the root of the type tree, convenient function when one
/// assumes that the type is elementary
type_node get_type_node(const type_tree& tt);

// takes in argument the arity of an operator (or procedure) and an
// index and convert that index such that if the arity is positive
// then the index is unchanged and if it's negative (i.e. arg_list is
// used) then it is converted to reach the last argument if it goes
// beyond the absolute arity
arity_t convert_index(arity_t arity, arity_t index);

// take in input an arity which can be explicit (using only input
// variables and no arg_list, or implicit (using all) and return the
// minimum number or input arguments the function with such arity can
// take
arity_t abs_min_arity(arity_t arity);

// return the type tree of the input argument of index i if the
// operator has arg_list(T) as last input argument then it returns
// always T past that index
const type_tree& argument_type_list_input_type(const type_tree_seq& atl,
                                               arity_t arity,
                                               arity_t index);

/// Return the arity of a vertex.
/// This is +1 if it is an argument, 0 if it is a constant, and -1 if
/// it is an arg list.  If vertex is one of the built-in functions,
/// then the returned arity is that of the function.
arity_t get_arity(const vertex& v);

// check if a type_tree is well formed
bool is_well_formed(const type_tree& tt);

// check if a combo_tree contains at least once every input arguments
// up to n and no input arguments above n it is assumed this function
// is only called with n>=0
bool does_contain_all_arg_up_to(const combo_tree& tr, arity_t n);

/// Return the sum of the arities of all the leaves of the tree.
/// The arity of a leaf is measured with get_arity(Vertex&), above.
/// Roughly speaking, this returns the total number of arguemnts in
/// the combo tree.
///
/// No type checking is performed.
arity_t infer_arity(const combo_tree& tr);

/// Return the highest argument index of combo tree tr;
/// for instance if tr == +($1 $5) it returns 5
arity_t explicit_arity(const combo_tree& tr);

/// Return the type tree of the output given the function signature ty
type_tree     get_signature_output(const type_tree& ty);

/// Return a sequence of type trees of the inputs of the signature ty
/// return a vector of the types of each input argument, given a
/// reduced signature (i.e. a lambda).  If there is an arg_list(T)
/// as the last argument, then the last element of the vector is
/// filled directly with T.
type_tree_seq get_signature_inputs(const type_tree& ty);

/// Given the types of the inputs and the output, create
/// a corresponding function signature, that is, a lambda type tree 
/// with that signature.  So, for example:
///      gen_signature({id::contin_type,  id::contin_type}, id::boolean_type)
/// returns
///     ->(contin contin boolean)
type_tree gen_signature(const std::vector<type_node>& itypes, type_node otype);

/// As above, except using a vector of input types.
type_tree gen_signature(const type_tree_seq& inputs, const type_tree& output);

/// For uniform input types, given the input arity, and the input type,
/// together with the output, create a corresponding function signature,
/// that is, a lambda type tree with that signature.  So, for example:
///      gen_signature(id::contin_type, id::boolean_type, 3)
/// returns
///     ->(contin contin contin boolean)
type_tree gen_signature(type_node itype, type_node otype, arity_t arity);

/// Same as above, but taking trees not nodes.
type_tree gen_signature(const type_tree& itype, const type_tree& otype,
                        arity_t arity);

/// Same as above, but using the same type for input and output.
type_tree gen_signature(type_node iotype, arity_t arity);
type_tree gen_signature(const type_tree& iotype, arity_t arity);


} // ~namespace combo
} // ~namespace opencog

namespace std {

std::ostream& operator<<(std::ostream&, const opencog::combo::type_node&);
std::istream& operator>>(std::istream&, opencog::combo::type_node&);

} // ~namespace std

#endif
