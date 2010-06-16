/** table.h --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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


#ifndef _OPENCOG_TABLE_H
#define _OPENCOG_TABLE_H

#include "eval.h"

namespace combo
{

//------------------------------------------------------------------------
// Truth table
//------------------------------------------------------------------------

class truth_table : public std::vector<bool>
{
public:
    typedef std::vector<bool> super;

    truth_table() { }
    template<typename It>
    truth_table(It from, It to) : super(from, to) { }
    template<typename T>
    truth_table(const opencog::tree<T>& t, int arity, opencog::RandGen& rng)
            : super(opencog::power(2, arity)) {
        populate(t, arity, rng);
    }
    template<typename T>
    truth_table(const opencog::tree<T>& t, opencog::RandGen& rng) {
        int a = arity(t);
        this->resize(opencog::power(2, a));
        populate(t, a, rng);
    }

    template<typename Func>
    truth_table(const Func& f, int arity, opencog::RandGen& rng)
        : super(opencog::power(2, arity)) {
        iterator it = begin();
        for (int i = 0;it != end();++i, ++it) {
            std::vector<bool> v(arity);
            for (int j = 0;j < arity;++j)
                v[j] = (i >> j) % 2;
            (*it) = f(v.begin(), v.end());
        }
    }

    /*
      this operator allows to access quickly to the results of a
      truth_table. [from, to) points toward a chain of boolean describing
      the inputs of the function coded into the truth_table and
      the operator returns the results.
    */
    template<typename It>
    bool operator()(It from,It to) {
        const_iterator it = begin();
        for (int i = 1;from != to;++from, i = i << 1)
            if (*from)
                it += i;
        return *it;
    }

    size_type hamming_distance(const truth_table& other) const;
protected:
    template<typename T>
    void populate(const opencog::tree<T>& tr,
                  int arity, opencog::RandGen& rng) {
        iterator it = begin();
        for (int i = 0;it != end();++i, ++it) {
            for (int j = 0;j < arity;++j)
                binding(j + 1) = bool_to_vertex((i >> j) % 2);
            (*it) = (eval(rng, tr) == id::logical_true);
        }
    }
};

//-------------------------------------------------------------------------
// contin table
//-------------------------------------------------------------------------

//shorthands used by class RndNumTable and contin_table
typedef std::vector<contin_t> contin_vector;
typedef contin_vector::iterator cv_it;
typedef contin_vector::const_iterator const_cv_it;
typedef std::vector<contin_vector> contin_matrix;
typedef contin_matrix::iterator cm_it;
typedef contin_matrix::const_iterator const_cm_it;

/*
  class RndNumTable
    matrix of randomly generated contin_t of sample_count rows and arity columns
*/
class RndNumTable : public contin_matrix
{
public:
    //constructor
    RndNumTable() {}
    RndNumTable(int sample_count, int arity, opencog::RandGen& rng,
                double max_randvalue = 1.0, double min_randvalue = -1.0);
};

/*
  class contin_table
    contains sample_count evaluations obtained by evaluating t, a tree, over
    a RndNumTable rnt.
    assumption : t has only contin inputs and output
*/
class contin_table : public contin_vector   //a column of results
{
public:
    //typedef contin_vector super;

    //constructors
    contin_table() { }
    contin_table(const combo_tree& t, const RndNumTable& rnt, opencog::RandGen& rng);
    template<typename Func>
    contin_table(const Func& f, const RndNumTable& rnt) {
        foreach(const contin_vector& v, rnt)
            push_back(f(v.begin(), v.end()));
    }

    //equality operator
    bool operator==(const contin_table& ct) const;
    bool operator!=(const contin_table& ct) const {
        return !operator==(ct);
    }

    contin_t abs_distance(const contin_table& other) const;
    contin_t sum_squared_error(const contin_table& other) const;
};

//-------------------------------------------------------------------------
// Mixed table
//-------------------------------------------------------------------------

class mixed_table
{
    typedef type_tree::iterator type_pre_it;
    typedef type_tree::sibling_iterator type_sib_it;
    //Vector of bool or contin, depending on the output type of combo_tree
    //size = 2^|boolean inputs| * _rnt.size()
    //each slice of rnt.size() corresponds to a particular boolean input
    //all possible boolean inputs are enumerated in lexico order as for
    //truth_table
    //each element in a slice of rnt.size() corresponds to the result of
    //a particular contin input, all obtained from rnt
    std::vector<variant<bool, contin_t> > _vt;
    //NOTE : can be a bit optimized by using
    //variant<std::vector<bool>,std::vector<contin_t> > _vt;
    //instead

    int _contin_arg_count; //number of contin arguments
    int _bool_arg_count; //number of boolean arguments

    boost::unordered_map<int, int> _arg_map; //link the argument index with the
    //boolean or contin index, that is
    //for instance if the inputs are
    //(bool, contin, contin, bool, bool)
    //the the map is
    //(<0,0>,<1,0>,<2,1>,<3,1>,<4,2>)
    std::vector<int> _bool_arg; //link the ith bool arg with arg index
    std::vector<int> _contin_arg; //link the ith contin arg with arg index
    type_tree _prototype;
    type_node _output_type;

    RndNumTable _rnt;

    //take a prototype, set _prototype, _contin_arg_count, _bool_arg_count
    //_arg_map, _bool_arg, _contin_arg and _output_type
    void set_prototype(const type_tree& prototype) {
        _contin_arg_count = 0;
        _bool_arg_count = 0;
        _prototype = prototype;
        type_pre_it proto_it = prototype.begin();
        int arg_idx = 0;
        for (type_sib_it i = proto_it.begin();
             i != proto_it.end(); ++i, ++arg_idx) {
            if (type_pre_it(i) == _prototype.last_child(proto_it)) {
                _output_type = *i;
            } else {
                if (*i == id::boolean_type) {
                    std::pair<int, int> p(arg_idx, _bool_arg_count);
                    _arg_map.insert(p);
                    _bool_arg.push_back(arg_idx);
                    _bool_arg_count++;
                } else if (*i == id::contin_type) {
                    std::pair<int, int> p(arg_idx, _contin_arg_count);
                    _arg_map.insert(p);
                    _contin_arg.push_back(arg_idx);
                    _contin_arg_count++;
                }
            }
        }
    }

public:
    //constructors
    mixed_table() {}
    mixed_table(const combo_tree& tr, const RndNumTable& rnt,
                const type_tree& prototype, opencog::RandGen& rng) {
        _rnt = rnt;
        if (prototype.empty()) {
            type_tree inferred_proto = infer_type_tree(tr);
            set_prototype(inferred_proto);
        } else set_prototype(prototype);
        //populate _vt
        //works only for _bool_arg_count < 64
        unsigned long int bool_table_size = 1 << _bool_arg_count;
        for (unsigned long int i = 0; i < bool_table_size; ++i) {
            for (int bai = 0; bai < _bool_arg_count; ++bai) //bool arg index
                binding(_bool_arg[bai] + 1) = bool_to_vertex((i >> bai) % 2);
            for (const_cm_it si = rnt.begin(); si != rnt.end(); ++si) {
                int cai = 0; //contin arg index
                for (const_cv_it j = (*si).begin(); j != (*si).end(); ++j, ++cai)
                    binding(_contin_arg[cai] + 1) = *j;
                vertex e = eval_throws(rng, tr);
                _vt.push_back(is_boolean(e) ? vertex_to_bool(e) : get_contin(e));
            }
        }
    }
    //access mesthods
    const std::vector<variant<bool, contin_t> >& get_vt() const {
        return _vt;
    }

    boost::unordered_map<int, int> & get_arg_idx_map()  {
		return _arg_map;
    }

    //equality operator
    bool operator==(const mixed_table& mt) const {
        std::vector<variant<bool, contin_t> >::const_iterator il = _vt.begin();
        for (std::vector<variant<bool, contin_t> >::const_iterator
                ir = mt._vt.begin(); ir != mt._vt.end(); ++il, ++ir) {
            if (boost::get<bool>(&(*il))) {
                if (boost::get<bool>(*il) != boost::get<bool>(*ir))
                    return false;
            } else if (!isApproxEq(boost::get<contin_t>(*il),
                                   boost::get<contin_t>(*ir)))
                return false;
        }
        return true;
    }
    bool operator!=(const mixed_table& mt) const {
        return !operator==(mt);
    }
};



//WARNING : should be in eval_action.h
//but could not do that due to dependency issues
//-------------------------------------------------------------------------
// mixed_action table
//-------------------------------------------------------------------------

//this table permits to represent inputs and outputs of combo trees
//that have boolean, continuous, action_result arguments and/or output

class mixed_action_table
{
    typedef type_tree::iterator type_pre_it;
    typedef type_tree::sibling_iterator type_sib_it;
    //Vector of bool or contin, depending on the output type of combo_tree
    //size = 2^|boolean+action_result inputs| * _rnt.size()
    //each slice of rnt.size() corresponds to a particular
    //boolean+action_result input, all possible boolean+action_result inputs
    //are enumerated in lexico order as for truth_table
    //each element in a slice of rnt.size() corresponds to the result of
    //a particular contin input, all obtained from rnt
    std::vector<variant<bool, contin_t> > _vt;
    //NOTE : can be a bit optimized by using
    //variant<std::vector<bool>,std::vector<contin_t> > _vt;
    //instead

    int _bool_arg_count; //number of boolean arguments
    int _contin_arg_count; //number of contin arguments
    int _action_arg_count; //number of action_result arguments

    boost::unordered_map<int, int> _arg_map; //link the argument index with the
    //boolean, action_result or contin
    //index, that is
    //for instance if the inputs are
    //(bool, contin, contin, bool, action)
    //the the map is
    //(<0,0>,<1,0>,<2,1>,<3,1>,<4,0>)
    std::vector<int> _bool_arg; //link the ith bool arg with arg index
    std::vector<int> _contin_arg; //link the ith contin arg with arg index
    std::vector<int> _action_arg; //link the ith action arg with arg index
    type_tree _prototype;
    type_node _output_type;

    RndNumTable _rnt;

    //take a prototype, set _prototype, _contin_arg_count, _bool_arg_count,
    //_action_arg_count, _arg_map, _bool_arg, _contin_arg, action_arg
    //and _output_type
    void set_prototype(const type_tree& prototype) {
        _contin_arg_count = 0;
        _bool_arg_count = 0;
        _action_arg_count = 0;
        _prototype = prototype;
        type_pre_it proto_it = prototype.begin();
        int arg_idx = 0;
        for (type_sib_it i = proto_it.begin();i != proto_it.end();++i, ++arg_idx) {
            if (type_pre_it(i) == _prototype.last_child(proto_it)) {
                _output_type = *i;
            } else {
                if (*i == id::boolean_type) {
                    std::pair<int, int> p(arg_idx, _bool_arg_count);
                    _arg_map.insert(p);
                    _bool_arg.push_back(arg_idx);
                    _bool_arg_count++;
                } else if (*i == id::contin_type) {
                    std::pair<int, int> p(arg_idx, _contin_arg_count);
                    _arg_map.insert(p);
                    _contin_arg.push_back(arg_idx);
                    _contin_arg_count++;
                } else if (*i == id::action_result_type) {
                    std::pair<int, int> p(arg_idx, _action_arg_count);
                    _arg_map.insert(p);
                    _action_arg.push_back(arg_idx);
                    _action_arg_count++;
                }
            }
        }
    }

public:
    //constructors
    mixed_action_table() {}
    mixed_action_table(const combo_tree& tr, const RndNumTable& rnt,
                       const type_tree& prototype, opencog::RandGen& rng) {
        _rnt = rnt;
        if (prototype.empty()) {
            type_tree inferred_proto = infer_type_tree(tr);
            set_prototype(inferred_proto);
        } else set_prototype(prototype);
        //populate _vt
        //WARNING : works only for _bool_arg_count + _action_arg_count <= 64
        int bool_action_arg_count = (_bool_arg_count + _action_arg_count);
        unsigned long int bool_action_table_size = 1 << bool_action_arg_count;
        for (unsigned long int i = 0; i < bool_action_table_size; ++i) {
            //bai stands for bool arg index
            for (int bai = 0; bai < _bool_arg_count; ++bai)
                binding(_bool_arg[bai] + 1) = bool_to_vertex((i >> bai) % 2);
            //aai stands for action arg index
            for (int aai = 0; aai < _action_arg_count; ++aai) {
                int arg_idx = _action_arg[aai] + 1;
                bool arg_val = (i >> (aai + _bool_arg_count)) % 2;
                binding(arg_idx) = (arg_val ? id::action_success : id::action_failure);
            }
            //contin populate
            for (const_cm_it si = rnt.begin();si != rnt.end();++si) {
                int cai = 0; //contin arg index
                for (const_cv_it j = (*si).begin(); j != (*si).end(); ++j, ++cai)
                    binding(_contin_arg[cai] + 1) = *j;
                vertex e = eval_throws(rng, tr);
                if (is_boolean(e))
                    _vt.push_back(vertex_to_bool(e));
                else if (is_action_result(e))
                    _vt.push_back(e == id::action_success ? true : false);
                else if (is_contin(e))
                    _vt.push_back(get_contin(e));
                //should never get to this part
                else OC_ASSERT(false,
                                       "should never get to this part.");
            }
        }
    }
    //access mesthods
    const std::vector<variant<bool, contin_t> >& get_vt() const {
        return _vt;
    }

    //equality operator
    bool operator==(const mixed_action_table& mat) const {
        std::vector<variant<bool, contin_t> >::const_iterator il = _vt.begin();
        for (std::vector<variant<bool, contin_t> >::const_iterator
                ir = mat._vt.begin(); ir != mat._vt.end(); ++il, ++ir) {
            if (boost::get<bool>(&(*il))) {
                if (boost::get<bool>(*il) != boost::get<bool>(*ir))
                    return false;
            } else if (!isApproxEq(boost::get<contin_t>(*il),
                                   boost::get<contin_t>(*ir)))
                return false;
        }
        return true;
    }
    bool operator!=(const mixed_action_table& mat) const {
        return !operator==(mat);
    }
};

} //~namespace combo

inline std::ostream& operator<<(std::ostream& out,
                                const combo::truth_table& tt)
{
    for (std::vector<bool>::const_iterator it = tt.begin();it != tt.end();++it)
        out << *it << " ";
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::contin_matrix& cm)
{
    for (combo::const_cm_it i = cm.begin(); i != cm.end(); ++i) {
        for (combo::const_cv_it j = (*i).begin(); j != (*i).end(); ++j) {
            out << *j << '\t';
        }
        out << std::endl;
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::RndNumTable& rnt)
{
    out << static_cast<combo::contin_matrix>(rnt);
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::contin_table& ct)
{
    for (combo::const_cv_it it = ct.begin(); it != ct.end(); ++it)
        out << *it << " ";
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::mixed_table& mt)
{
    const std::vector<boost::variant<bool, combo::contin_t> >& vt = mt.get_vt();
    for (unsigned i = 0; i != vt.size(); ++i)
        out << (boost::get<combo::contin_t>(&vt[i]) ?
                boost::get<combo::contin_t>(vt[i]) :
                boost::get<bool>(vt[i])) << " ";
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::mixed_action_table& mat)
{
    const std::vector<boost::variant<bool, combo::contin_t> >& vt = mat.get_vt();
    for (unsigned i = 0; i != vt.size(); ++i)
        out << (boost::get<combo::contin_t>(&vt[i]) ?
                boost::get<combo::contin_t>(vt[i]) :
                boost::get<bool>(vt[i])) << " ";
    return out;
}

namespace boost
{
inline size_t hash_value(const combo::truth_table& tt)
{
    return hash_range(tt.begin(), tt.end());
}
} //~namespace boost

#endif // _OPENCOG_TABLE_H
