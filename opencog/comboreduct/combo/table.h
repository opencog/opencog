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

#include <fstream>

#include <boost/iterator/counting_iterator.hpp>
#include <boost/tokenizer.hpp>

#include <opencog/util/RandGen.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/dorepeat.h>

#include "eval.h"
#include "vertex.h"

namespace combo
{

using namespace opencog;

///////////////////
// Generic table //
///////////////////

/**
 * Matrix of type T.
 * Rows represent samples.
 * Columns represent input variables
 * Optionally a list of labels (input variable names)
 *
 * It contains additional methods to ignore some variables, this is
 * done only for speeding up evaluation of data fitting objective
 * functions when variables in the function to evaluate are
 * scarce. The gain is noticable but not tremendous (if I remember no
 * more 10% depending on the case). If it's too ugly and constraining
 * to maintain I don't think it would be a big loss to remove that
 * optimization. Of course in the case where there are millions of
 * columns and only a few variables to evaluate the gain could be
 * tremendous.
 */
template<typename T>
class input_table {
public:
    typedef std::vector<T> VT;
    typedef std::vector<VT> MT;

    input_table() {}

    input_table(const MT& mt,
                std::vector<std::string>& il = std::vector<std::string>())
        : labels(il) {
        foreach(std::vector<T>& row, mt)
            push_back(row); // using push_back to update considered_args
    }

    // set input labels
    void set_labels(std::vector<std::string>& il) {
        labels = il;
    }

    // set binding prior calling the combo evaluation, ignoring inputs
    // to be ignored
    void set_binding(const std::vector<T>& args) const {
        for(std::set<arity_t>::const_iterator cit = considered_args.begin();
            cit != considered_args.end(); cit++)
            binding(*cit + 1) = args[*cit];
    }
    arity_t get_arity() const {
        return matrix.front().size();
    }
    // set the inputs to ignore, if all are ignored an assert is
    // raised. That method resets arguments.
    void set_ignore_args(const vertex_set& ignore_args) {
        std::set<arity_t> ii;
        foreach(const vertex& ia, ignore_args)
            if(is_argument(ia))
                ii.insert(get_argument(ia).abs_idx_from_zero());
        set_ignore_args_from_zero(ii);
    }
    // like above but takes a set of indices instead of vertex
    void set_ignore_args_from_zero(const std::set<arity_t>& ignore_idxs) {
        considered_args.clear();
        for(arity_t idx = 0; idx < get_arity(); idx++)
            if(ignore_idxs.find(idx) == ignore_idxs.end())
                considered_args.insert(idx);
        OC_ASSERT(!considered_args.empty(), "You cannot ignore all arguments");
    }
    // set the inputs to consider. That method resets arguments.
    void set_consider_args(const vertex_set& consider_args) {
        considered_args.clear();
        foreach(const vertex& arg, consider_args)
            considered_args.insert(get_argument(arg).abs_idx_from_zero());
    }
    // like above but take the indices of columns instead of arguments
    void set_consider_args_from_zero(const std::set<arity_t>& consider_idx) {
        considered_args = consider_idx;
    }

    vertex_set get_considered_args() {
        vertex_set res;
        foreach(arity_t idx, considered_args)
            res.insert(argument(argument::idx_from_zero_to_idx(idx)));
        return res;
    }

    // the reverse of get_considered_args
    vertex_set get_ignore_args() {
        vertex_set res;
        for(arity_t idx = 0; idx < get_arity(); idx++)
            if(considered_args.find(idx) == considered_args.end())
                res.insert(argument(argument::idx_from_zero_to_idx(idx)));
        return res;
    }

    // returns the set of column indices corresponding to the
    // considered arguments
    const std::set<arity_t>& get_considered_args_from_zero() const {
        return considered_args;
    }

    // STL
    typedef typename MT::iterator iterator;
    typedef typename MT::const_iterator const_iterator;
    typedef typename MT::size_type size_type;
    typedef typename MT::value_type value_type;
    MT& get_matrix() { return matrix;}
    const MT& get_matrix() const {return matrix;}
    std::vector<std::string>& get_labels() { return labels; }
    const std::vector<std::string>& get_labels() const { return labels; }
    VT& operator[](size_t i) { return get_matrix()[i]; }
    const VT& operator[](size_t i) const { return get_matrix()[i]; }
    iterator begin() {return matrix.begin();}
    iterator end() {return matrix.end();}
    const_iterator begin() const {return matrix.begin();}
    const_iterator end() const {return matrix.end();}
    bool operator==(const input_table<T>& rhs) const {
        return get_matrix() == rhs.get_matrix()
            && get_labels() == rhs.get_labels();
    }
    
    // Warning: this method also update considered_args, by assuming
    // all are considered
    void push_back(const std::vector<T>& t) {
        matrix.push_back(t);
        if(considered_args.empty())
            considered_args.insert(boost::counting_iterator<arity_t>(0),
                                   boost::counting_iterator<arity_t>(get_arity()));
    }
    size_type size() const {return matrix.size();}
    iterator erase(iterator it) {return matrix.erase(it);}
protected:
    MT matrix;
    std::vector<std::string> labels; // list of input labels
    // the set of arguments (represented directly as column indices)
    // to consider
    std::set<arity_t> considered_args; 
};

template<typename T>
class output_table : public std::vector<T> {
    typedef std::vector<T> super;
public:
    output_table() {}
    output_table(const super& ot, std::string& ol = "") 
        : super(ot), label(ol) {}

    void set_label(const std::string& ol) { label = ol; }
    std::string& get_label() { return label; }
    const std::string& get_label() const { return label; }

    // STL
    bool operator==(const output_table<T>& rhs) const {
        return 
            static_cast<const super&>(*this) == static_cast<const super&>(rhs)
            && rhs.get_label() == label;
    }
private:
    std::string label; // output label
};

/////////////////
// Truth table //
/////////////////

//shorthands used by class contin_table_inputs and contin_table
typedef std::vector<bool> bool_vector;
typedef bool_vector::iterator bv_it;
typedef bool_vector::const_iterator bv_cit;
typedef std::vector<bool_vector> bool_matrix;
typedef bool_matrix::iterator bm_it;
typedef bool_matrix::const_iterator bm_cit;

/**
 * complete truth table, it contains only the outputs, the inputs are
 * assumed to be ordered in the conventional way, for instance if
 * there are 2 inputs, the output is ordered as follows:
 *
 * +--+--+--------------+
 * |#1|#2|Output        |
 * +--+--+--------------+
 * |F |F |truth_table[0]|
 * +--+--+--------------+
 * |T |F |truth_table[1]|
 * +--+--+--------------+
 * |F |T |truth_table[2]|
 * +--+--+--------------+
 * |T |T |truth_table[3]|
 * +--+--+--------------+
 */
class truth_table : public bool_vector
{
public:
    typedef bool_vector super;

    truth_table() : _rng(NULL) { }
    template<typename It>
    truth_table(It from, It to) : super(from, to), _rng(NULL) { }
    template<typename T>
    truth_table(const tree<T>& tr, arity_t arity)
        : super(pow2(arity)), _arity(arity), _rng(NULL) {
        populate(tr);
    }
    template<typename T>
    truth_table(const tree<T>& tr) {
        _arity = arity(tr);
        _rng = NULL;
        this->resize(pow2(_arity));
        populate(tr);
    }

    template<typename Func>
    truth_table(const Func& f, arity_t arity)
        : super(pow2(arity)), _arity(arity), _rng(NULL) {
        iterator it = begin();
        for (int i = 0; it != end(); ++i, ++it) {
            bool_vector v(_arity);
            for (arity_t j = 0;j < _arity;++j)
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

    /**
     * compute the truth table of tr and compare it to self. This
     * method is optimized so that if there are not equal it can be
     * detected before calculating the entire table.
     */
    bool same_truth_table(const combo_tree& tr) const;
protected:
    template<typename T>
    void populate(const tree<T>& tr) {
        iterator it = begin();
        for (int i = 0; it != end(); ++i, ++it) {
            for (int j = 0; j < _arity; ++j)
                binding(j + 1) = bool_to_vertex((i >> j) % 2);
            (*it) = (eval(*_rng, tr) == id::logical_true);
        }
    }
    arity_t _arity;
    RandGen* _rng; // _rng is dummy and not used anyway
};


/**
 * truth_table_inputs, matrix of booleans, each row corresponds to a
 * possible vector input
 */
class truth_table_inputs : public input_table<bool> {
public:
    // set binding prior calling the combo evaluation, ignoring inputs
    // to be ignored
    void set_binding(const std::vector<bool>& args) const {
        for(std::set<arity_t>::const_iterator cit = considered_args.begin();
            cit != considered_args.end(); cit++)
            binding(*cit + 1) = bool_to_vertex(args[*cit]);
    }
};

/**
 * partial_truth_table, column of result of a corresponding truth_table_inputs
 */
struct partial_truth_table : public output_table<bool> {
    partial_truth_table() {}
    partial_truth_table(const bool_vector& bv, std::string ol = "")
        : output_table<bool>(bv, ol) {}
    partial_truth_table(const combo_tree& tr, const truth_table_inputs& tti,
                        RandGen& rng);
    
};


//////////////////
// contin table //
//////////////////

//shorthands used by class contin_table_inputs and contin_table
typedef std::vector<contin_t> contin_vector;
typedef contin_vector::iterator cv_it;
typedef contin_vector::const_iterator const_cv_it;
typedef std::vector<contin_vector> contin_matrix;
typedef contin_matrix::iterator cm_it;
typedef contin_matrix::const_iterator const_cm_it;

/*
  class contin_table_inputs
    matrix of randomly generated contin_t of sample_count rows and arity columns
*/
class contin_input_table : public input_table<contin_t>
{
public:
    // constructors
    contin_input_table() {}
    contin_input_table(int sample_count, int arity, RandGen& rng,
                       double max_randvalue = 1.0, double min_randvalue = -1.0);
};

/*
  class contin_table
    contains sample_count evaluations obtained by evaluating t, a tree, over
    a RndNumTable cti.
    assumption : t has only contin inputs and output
*/
class contin_table : public output_table<contin_t>   //a column of results
{
public:
    //typedef contin_vector super;

    //constructors
    contin_table() {}
    contin_table(const contin_vector& cv, std::string ol = "") 
        : output_table<contin_t>(cv, ol) {}
    contin_table(const combo_tree& tr, const contin_input_table& cti,
                 RandGen& rng);
    template<typename Func>
    contin_table(const Func& f, const contin_input_table& cti) {
        foreach(const contin_vector& v, cti.get_matrix())
            push_back(f(v.begin(), v.end()));
    }

    //equality operator
    bool operator==(const contin_table& ct) const;
    bool operator!=(const contin_table& ct) const {
        return !operator==(ct);
    }
    // total of the absolute distance of each value
    contin_t abs_distance(const contin_table& other) const;
    // total of the squared error of each value
    contin_t sum_squared_error(const contin_table& other) const;
    // mean of the squared error of each value
    contin_t mean_squared_error(const contin_table& other) const;
    // sqrt of mean_squared_error
    contin_t root_mean_square_error(const contin_table& other) const;
};


/////////////////
// Mixed table //
/////////////////

class mixed_table
{
    typedef type_tree::iterator type_pre_it;
    typedef type_tree::sibling_iterator type_sib_it;
    //Vector of bool or contin, depending on the output type of combo_tree
    //size = 2^|boolean inputs| * _cti.size()
    //each slice of cti.size() corresponds to a particular boolean input
    //all possible boolean inputs are enumerated in lexico order as for
    //truth_table
    //each element in a slice of cti.size() corresponds to the result of
    //a particular contin input, all obtained from cti
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

    contin_input_table _cti;

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
    mixed_table(const combo_tree& tr, const contin_input_table& cti,
                const type_tree& prototype, RandGen& rng) {
        _cti = cti;
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
            for (const_cm_it si = cti.begin(); si != cti.end(); ++si) {
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
    //size = 2^|boolean+action_result inputs| * _cti.size()
    //each slice of cti.size() corresponds to a particular
    //boolean+action_result input, all possible boolean+action_result inputs
    //are enumerated in lexico order as for truth_table
    //each element in a slice of cti.size() corresponds to the result of
    //a particular contin input, all obtained from cti
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

    contin_input_table _cti;

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
    mixed_action_table(const combo_tree& tr, const contin_input_table& cti,
                       const type_tree& prototype, RandGen& rng) {
        _cti = cti;
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
            for (const_cm_it si = cti.begin();si != cti.end();++si) {
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

//////////////////
// istreamTable //
//////////////////

/**
 * remove the carriage return (for DOS format)
 */
void removeCarriageReturn(std::string& str);

/**
 * remove non ASCII char at the begining of the string
 */
void removeNonASCII(std::string& str);

/**
 * Return true if the next chars in 'in' correspond to carriage return
 * (support UNIX and DOS format) and advance in of the checked chars.
 */
bool checkCarriageReturn(std::istream& in);
 
/**
 * Return the arity of the table provided in istream (by counting the
 * number of elements of the first line).
 */
arity_t istreamArity(std::istream& in);
/**
 * Helper, like above but given the file name instead of istream
 */
arity_t dataFileArity(const std::string& dataFileName);

/**
 * Infer the type of elements of the data file
 */
type_node inferDataType(const std::string& dataFileName);

/**
 * take a line and return the input vector and output.
 * Used by istreamTable.
 * Please note that it may modify line to be Unix compatible
 */
template<typename T>
void tokenizeRow(std::string& line, std::vector<T>& input_vec, T& output) {
    typedef boost::escaped_list_separator<char> seperator;
    typedef boost::tokenizer<seperator> tokenizer;
    typedef tokenizer::const_iterator tokenizer_cit;

    // remove weird symbols and carriage return symbol (for DOS files)
    removeNonASCII(line);
    removeCarriageReturn(line);

    // tokenize line
    seperator sep("\\", ", \t", "\"");
    tokenizer tok(line, sep);
    for(tokenizer_cit it = tok.begin(); it != tok.end(); it++) {
        //std::cout << "Token = " << *it << std::endl;
        if(++tokenizer_cit(it) != tok.end())
            input_vec.push_back(boost::lexical_cast<T>(*it));
        else output = boost::lexical_cast<T>(*it);
    }
}
// helper
template<typename T>
std::pair<std::vector<T>, T> tokenizeRow(std::string& line) {
    std::vector<T> input_vec;
    T output;
    tokenizeRow(line, input_vec, output);
    return std::make_pair(input_vec, output);
}
template<typename T>
std::vector<T> tokenizeRowVec(std::string& line) {
    std::pair<std::vector<T>, T> p = tokenizeRow<T>(line);
    p.first.push_back(p.second);
    return p.first;
}

/**
 * template to fill an input table (IT) and output table (OT) of type
 * T, given a DSV (delimiter-seperated values) file format, where
 * delimiters are ',', ' ' or '\t'.
 * 
 * It is assumed that each row have the same number of columns, if not
 * an assert is raised.
 */
template<typename IT, typename OT, typename T>
std::istream& istreamTable(std::istream& in, IT& table_inputs, OT& output_table) {
    std::string line;

    ///////////////////////////////////////////////////
    // first row, check if they are labels or values //
    ///////////////////////////////////////////////////
    getline(in, line);    
    std::vector<std::string> input_labels; // possibly labels
    std::string output_label; // possibly label
    tokenizeRow<std::string>(line, input_labels, output_label);
    try { // try to interpret then as values
        std::vector<T> input_vec;
        T output;
        foreach(std::string& s, input_labels)
            input_vec.push_back(boost::lexical_cast<T>(s));
        output = boost::lexical_cast<T>(output_label);
        // they are values so we add them
        table_inputs.push_back(input_vec);
        output_table.push_back(output);        
    } catch (boost::bad_lexical_cast &) { // not interpretable, they
                                          // must be labels
        table_inputs.set_labels(input_labels);
        output_table.set_label(output_label);
    }
    arity_t arity = input_labels.size();
    
    //////////////////////////////////////////
    // next rows, we assume they are values //
    //////////////////////////////////////////
    while (getline(in, line)) {
        // tokenize the line and fill the input vector and output
        std::pair<std::vector<T>, T> iop = tokenizeRow<T>(line);
        
        // check arity
        OC_ASSERT(arity == (arity_t)iop.first.size(),
                  "The row %u has %u columns while the first row has %d"
                  " columns, all rows should have the same number of"
                  " columns", output_table.size(), iop.first.size(), arity);
        
        // fill table
        table_inputs.push_back(iop.first);
        output_table.push_back(iop.second);
    }
    return in;
}
/**
 * like above but take an string (file name) instead of istream. If
 * the file name is not correct then an OC_ASSERT is raised.
 */
template<typename IT, typename OT, typename T>
void istreamTable(const std::string& file_name,
                  IT& table_inputs, OT& output_table) {
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    std::ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());
    istreamTable<IT, OT, T>(in, table_inputs, output_table);
}

//////////////////
// ostreamTable //
//////////////////

// output a data table in CSV format, not that ignored arguments are
// not printed
template<typename IT, typename OT>
std::ostream& ostreamTable(std::ostream& out, const IT& it, const OT& ot) {
    OC_ASSERT(it.size() == ot.size());
    const std::set<arity_t>& ca = it.get_considered_args_from_zero();
    // print labels
    if(!it.get_labels().empty() && !ot.get_label().empty()) {
        foreach(size_t col, ca)
            out << it.get_labels()[col] << ",";
        out << ot.get_label() << std::endl;
    }
    // print data
    for(size_t row = 0; row < it.size(); row++) {
        foreach(size_t col, ca)
            out << it[row][col] << ",";
        out << ot[row] << std::endl;
    }
    return out;
}
// like above but takes the file name where to write the table
template<typename IT, typename OT>
void ostreamTable(const std::string& file_name, const IT& it, const OT& ot) {
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    std::ofstream out(file_name.c_str());
    OC_ASSERT(out.is_open(), "Could not open %s", file_name.c_str());
    ostreamTable(out, it, ot);
}

/**
 * template to subsample input and output tables, after subsampling
 * the table have size min(nsamples, *table.size())
 */
template<typename IT, typename OT>
void subsampleTable(IT& input_table, OT& output_table,
                    unsigned int nsamples, RandGen& rng) {
    OC_ASSERT(input_table.size() == output_table.size());
    if(nsamples < output_table.size()) {
        unsigned int nremove = output_table.size() - nsamples;
        dorepeat(nremove) {
            unsigned int ridx = rng.randint(output_table.size());
            input_table.erase(input_table.begin()+ridx);
            output_table.erase(output_table.begin()+ridx);
        }
    }
}
/**
 * like above but subsample only the input table
 */
template<typename IT>
void subsampleTable(IT& table_inputs, unsigned int nsamples, RandGen& rng) {
    if(nsamples < table_inputs.size()) {
        unsigned int nremove = table_inputs.size() - nsamples;
        dorepeat(nremove) {
            unsigned int ridx = rng.randint(table_inputs.size());
            table_inputs.erase(table_inputs.begin()+ridx);
        }
    }
}

/**
 * if the data file has a first row with labels
 */
std::vector<std::string> read_data_file_labels(const std::string& file);

std::ifstream* open_data_file(const std::string& fileName);

} // ~namespace combo

template<typename T>
inline std::ostream& operator<<(std::ostream& out,
                                const std::vector<std::vector<T> >& mt)
{
    foreach(const std::vector<T>& row, mt) {
        opencog::ostreamContainer(out, row, ",");
        out << std::endl;
    }
    return out;
}

template<typename T>
inline std::ostream& operator<<(std::ostream& out,
                                const combo::input_table<T>& it)
{
    if(!it.get_labels().empty()) {
        opencog::ostreamContainer(out, it.get_labels(), ",");
        out << std::endl;
    }
    out << it.get_matrix();
    return out;
}

template<typename T>
inline std::ostream& operator<<(std::ostream& out,
                                const combo::output_table<T>& ot)
{
    if(!ot.get_label().empty())
        out << ot.get_label() << std::endl;
    return opencog::ostreamContainer(out, ot, "\n");
}

inline std::ostream& operator<<(std::ostream& out,
                                const combo::truth_table& tt)
{
    return opencog::ostreamContainer(out, tt);
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
