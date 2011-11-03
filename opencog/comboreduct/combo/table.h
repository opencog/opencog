/** table.h --- 
 *
 * Copyright (C) 2010 OpenCog Foundation
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
#include "common_def.h"

namespace opencog { namespace combo {

using boost::variant;

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
 * return the position of the target in the DSV data file fileName. If
 * none raise an assert.
 */
int findTargetFeaturePosition(const std::string& fileName,
                              const std::string& target);

/**
 * take a row, modify it to be Unix format compatible and return a
 * tokenizer of it using as sperator the characters ',', ' ' or '\t'.
 */
boost::tokenizer<boost::char_separator<char>> get_row_tokenizer(std::string& line);

/**
 * take a line and return a vector containing the elements parsed.
 * Used by istreamTable. Please note that it may modify line to be
 * Unix compatible.
 */
template<typename T>
std::vector<T> tokenizeRow(std::string& line) {
    boost::tokenizer<boost::char_separator<char> > tok = get_row_tokenizer(line);
    std::vector<T> res;
    foreach(const std::string& t, tok)
        res.push_back(boost::lexical_cast<T>(t));
    return res;
}
// Like above but split the result into an vector (the inputs) and an
// element (the output) given that the output is at position pos. If
// pos < 0 then the position is the last. If pos >= 0 then the
// position is as usual (0 is the first, 1 is the second, etc).
// If pos is out of range then an assert is raised.
template<typename T>
std::pair<std::vector<T>, T> tokenizeRowIO(std::string& line, int pos = -1) {
    boost::tokenizer<boost::char_separator<char> > tok = get_row_tokenizer(line);
    std::vector<T> inputs;
    T output;
    int i = 0;
    foreach(const std::string& t, tok) {
        if(i++ != pos)
            inputs.push_back(boost::lexical_cast<T>(t));
        else output = boost::lexical_cast<T>(t);
    }
    if(pos < 0) {
        output = inputs.back();
        inputs.pop_back();
    }
    // the following assert is to garanty that the output has been filled
    OC_ASSERT((int)inputs.size() == i-1);
    return {inputs, output};
}

/**
 * template to fill an input table (IT) and output table (OT) of type
 * T, given a DSV (delimiter-seperated values) file format, where
 * delimiters are ',', ' ' or '\t'.
 * 
 * It is assumed that each row have the same number of columns, if not
 * an assert is raised.
 *
 * pos specifies the position of the output, by default the last one.
 */
template<typename IT, typename OT>
std::istream& istreamTable(std::istream& in, IT& input_table, OT& output_table,
                           int pos = -1) {
    typedef typename OT::value_type T;
    std::string line;

    ///////////////////////////////////////////////////
    // first row, check if they are labels or values //
    ///////////////////////////////////////////////////
    getline(in, line);    
    std::pair<std::vector<std::string>, std::string> ioh =
        tokenizeRowIO<std::string>(line, pos);
    try { // try to interpret then as values
        std::vector<T> inputs;
        foreach(auto& s, ioh.first)
            inputs.push_back(boost::lexical_cast<T>(s));
        T output = boost::lexical_cast<T>(ioh.second);
        // they are values so we add them
        input_table.push_back(inputs);
        output_table.push_back(output);
    } catch (boost::bad_lexical_cast &) { // not interpretable, they
                                          // must be labels
        input_table.set_labels(ioh.first);
        output_table.set_label(ioh.second);
    }
    arity_t arity = ioh.first.size();
    
    //////////////////////////////////////////
    // next rows, we assume they are values //
    //////////////////////////////////////////
    while (getline(in, line)) {
        // tokenize the line and fill the input vector and output
        std::pair<std::vector<T>, T> io = tokenizeRowIO<T>(line, pos);
        
        // check arity
        OC_ASSERT(arity == (arity_t)io.first.size(),
                  "The row %u has %u columns while the first row has %d"
                  " columns, all rows should have the same number of"
                  " columns", output_table.size(), io.first.size(), arity);
        
        // fill table
        input_table.push_back(io.first);
        output_table.push_back(io.second);
    }
    return in;
}
/**
 * like above but take an string (file name) instead of istream. If
 * the file name is not correct then an OC_ASSERT is raised.
 */
template<typename IT, typename OT>
void istreamTable(const std::string& file_name,
                  IT& input_table, OT& output_table, int pos = -1) {
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    std::ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());
    istreamTable(in, input_table, output_table, pos);
}

//////////////////
// ostreamTable //
//////////////////

// output the header of a data table in CSV format.
template<typename IT, typename OT>
std::ostream& ostreamTableHeader(std::ostream& out, const IT& it, const OT& ot) {
    ostreamContainer(out, it.get_labels(), ",", "", ",");
    out << ot.get_label() << std::endl;
    return out;
}

// output a data table in CSV format, note that ignored arguments are
// not printed
template<typename IT, typename OT>
std::ostream& ostreamTable(std::ostream& out, const IT& it, const OT& ot) {
    // print header
    ostreamTableHeader(out, it, ot);
    // print data
    OC_ASSERT(it.size() == ot.size());
    for(size_t row = 0; row < it.size(); ++row) {
        for(arity_t col = 0; col < it.get_arity(); col++)
            out << it[row][col] << ",";
        out << ot[row] << std::endl;
    }
    return out;
}
// like above but take a table instead of a input and output table
template<typename Table>
std::ostream& ostreamTable(std::ostream& out, const Table& table) {
    return ostreamTable(out, table.input, table.output);
}

// like above but takes the file name where to write the table
template<typename IT, typename OT>
void ostreamTable(const std::string& file_name, const IT& it, const OT& ot) {
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    std::ofstream out(file_name.c_str());
    OC_ASSERT(out.is_open(), "Could not open %s", file_name.c_str());
    ostreamTable(out, it, ot);
}
// like above but take a table instead of a input and output table
template<typename Table>
void ostreamTable(const std::string& file_name, const Table& table) {
    ostreamTable(file_name, table.input, table.output);
}

/**
 * template to subsample input and output tables, after subsampling
 * the table have size min(nsamples, *table.size())
 */
template<typename IT, typename OT>
void subsampleTable(IT& it, OT& ot, unsigned int nsamples, RandGen& rng) {
    OC_ASSERT(it.size() == ot.size());
    if(nsamples < ot.size()) {
        unsigned int nremove = ot.size() - nsamples;
        dorepeat(nremove) {
            unsigned int ridx = rng.randint(ot.size());
            it.erase(it.begin()+ridx);
            ot.erase(ot.begin()+ridx);
        }
    }
}
/**
 * like above but subsample only the input table
 */
template<typename IT>
void subsampleTable(IT& input_table, unsigned int nsamples, RandGen& rng) {
    if(nsamples < input_table.size()) {
        unsigned int nremove = input_table.size() - nsamples;
        dorepeat(nremove) {
            unsigned int ridx = rng.randint(input_table.size());
            input_table.erase(input_table.begin()+ridx);
        }
    }
}

///////////////////
// Generic table //
///////////////////

static const std::string default_input_label("i");

/**
 * Matrix of type T.
 * Rows represent samples.
 * Columns represent input variables
 * Optionally a list of labels (input variable names)
 */
template<typename T>
class input_table : public std::vector<std::vector<T> > {
    typedef std::vector<std::vector<T> > super;
public:
    input_table() {}
    input_table(const super& mt,
                std::vector<std::string> il = std::vector<std::string>())
        : super(mt), labels(il) {
    }
    // set input labels
    void set_labels(std::vector<std::string> il) {
        labels = il;
    }
    const std::vector<std::string>& get_labels() const {
        if(labels.empty() and !super::empty()) { // return default labels
            static const std::vector<std::string> dl(get_default_labels());
            return dl;
        } else return labels;
    }
    // like get_labels but filter accordingly to a container of arity_t
    template<typename F>
    std::vector<std::string> get_filtered_labels(const F& filter) {
        std::vector<std::string> res;
        foreach(arity_t a, filter)
            res.push_back(get_labels()[a]);
        return res;
    }
    // get binding map prior calling the combo evaluation
    binding_map get_binding_map(const std::vector<T>& args) const {
        binding_map bmap;
        for(arity_t i = 0; i < (arity_t)args.size(); ++i)
            bmap[i+1] = args[i];
        return bmap;
    }
    arity_t get_arity() const {
        return super::front().size();
    }
    bool operator==(const input_table<T>& rhs) const {
        return 
            static_cast<const super&>(*this) == static_cast<const super&>(rhs)
            && get_labels() == rhs.get_labels();
    }
    /// return a copy of the input table filtered according to a given
    /// container of arity_t
    template<typename F>
    input_table filter(const F& f) {
        input_table res;
        res.set_labels(get_filtered_labels(f));
        foreach(const typename super::value_type& row, *this) {
            std::vector<T> new_row;
            foreach(arity_t a, f)
                new_row.push_back(row[a]);
            res.push_back(new_row);
        }
        return res;
    }
protected:
    std::vector<std::string> labels; // list of input labels
private:
    std::vector<std::string> get_default_labels() const {
        std::vector<std::string> res;
        for(arity_t i = 1; i <= get_arity(); ++i)
            res.push_back(default_input_label 
                          + boost::lexical_cast<std::string>(i));
        return res;
    }
};

static const std::string default_output_label("output");

template<typename T>
class output_table : public std::vector<T> {
    typedef std::vector<T> super;
public:
    typedef T value_type;

    output_table(const std::string& ol = default_output_label)
        : label(ol) {}
    output_table(const super& ot, const std::string& ol = default_output_label)
        : super(ot), label(ol) {}

    void set_label(const std::string& ol) { label = ol; }
    std::string& get_label() { return label; }
    const std::string& get_label() const { return label; }
    bool operator==(const output_table<T>& rhs) const {
        return 
            static_cast<const super&>(*this) == static_cast<const super&>(rhs)
            && rhs.get_label() == label;
    }
private:
    std::string label; // output label
};

template<typename IT, typename OT>
struct table {
    typedef IT InputTable;
    typedef OT OutputTable;
    typedef typename IT::value_type value_type;

    table() {}
    table(std::istream& in, int pos = -1) {
        istreamTable(in, input, output, pos);
    }
    table(const std::string& file_name, int pos = -1) {
        istreamTable(file_name, input, output, pos);
    }
    size_t size() const { return input.size(); }
    arity_t get_arity() const { return input.get_arity(); }
    template<typename F> table filter(const F& f) {
        table res;
        res.input = input.filter(f);
        res.output = output;
        return res;
    }
    IT input;
    OT output;
};

/////////////////
// Truth table //
/////////////////

//shorthands used by class contin_input_table and contin_output_table
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
 * +--+--+-----------------------+
 * |#1|#2|Output                 |
 * +--+--+-----------------------+
 * |F |F |complete_truth_table[0]|
 * +--+--+-----------------------+
 * |T |F |complete_truth_table[1]|
 * +--+--+-----------------------+
 * |F |T |complete_truth_table[2]|
 * +--+--+-----------------------+
 * |T |T |complete_truth_table[3]|
 * +--+--+-----------------------+
 */
class complete_truth_table : public bool_vector
{
public:
    typedef bool_vector super;

    complete_truth_table() : _rng(NULL) { }
    template<typename It>
    complete_truth_table(It from, It to) : super(from, to), _rng(NULL) { }
    template<typename T>
    complete_truth_table(const tree<T>& tr, arity_t arity)
        : super(pow2(arity)), _arity(arity), _rng(NULL) {
        populate(tr);
    }
    template<typename T>
    complete_truth_table(const tree<T>& tr) {
        _arity = arity(tr);
        _rng = NULL;
        this->resize(pow2(_arity));
        populate(tr);
    }

    template<typename Func>
    complete_truth_table(const Func& f, arity_t arity)
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
      complete_truth_table. [from, to) points toward a chain of boolean describing
      the inputs of the function coded into the complete_truth_table and
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

    size_type hamming_distance(const complete_truth_table& other) const;

    /**
     * compute the truth table of tr and compare it to self. This
     * method is optimized so that if there are not equal it can be
     * detected before calculating the entire table.
     */
    bool same_complete_truth_table(const combo_tree& tr) const;
protected:
    template<typename T>
    void populate(const tree<T>& tr) {
        iterator it = begin();
        for (int i = 0; it != end(); ++i, ++it) {
            for (int j = 0; j < _arity; ++j)
                bmap[j + 1] = bool_to_vertex((i >> j) % 2);
            *it = eval_binding(*_rng, bmap, tr) == id::logical_true;
        }
    }
    arity_t _arity;
    RandGen* _rng; // _rng is dummy and not used anyway
    mutable binding_map bmap;
};


/**
 * truth_input_table, matrix of booleans, each row corresponds to a
 * possible vector input
 */
class truth_input_table : public input_table<bool> {
    typedef input_table<bool> super;
public:
    truth_input_table() {}
    truth_input_table(const super& it) : super(it) {}
    // set binding prior calling the combo evaluation, ignoring inputs
    // to be ignored
    binding_map get_binding_map(const std::vector<bool>& args) const {
        binding_map bmap;
        for(size_t i = 0; i < args.size(); ++i)
            bmap[i+1] = bool_to_vertex(args[i]);
        return bmap;
    }
};

/// Struct to contain a compressed truth table, for instance if
/// the following truth table is:
///
/// i1,i2,o
/// 1,0,1
/// 1,1,0
/// 1,0,1
/// 1,0,0
///
/// the compressed truth table is
///
/// i1,i2,o
/// 1,0,(1,2)
/// 1,1,(1,0)
///
/// that is the duplicated inputs are removed and the output is
/// replaced by a counter of the false ones and the true ones
/// respectively.
///
/// Note: this could generalized using a mapping<T, unsigned> instead
/// of std::pair for non-boolean case. Here with T = bool it's fine
/// though.
class ctruth_table : public std::map<std::vector<bool>, std::pair<unsigned, unsigned> > {
    typedef std::map<std::vector<bool>, std::pair<unsigned, unsigned> > super;
public:
    binding_map get_binding_map(const std::vector<bool>& args) const {
        binding_map bmap;
        for(size_t i = 0; i < args.size(); ++i)
            bmap[i+1] = bool_to_vertex(args[i]);
        return bmap;
    }
};

/**
 * truth_output_table, column of result of a corresponding truth_input_table
 */
struct truth_output_table : public output_table<bool> {
    truth_output_table(const std::string& ol = default_output_label)
        : output_table<bool>(ol) {}
    truth_output_table(const bool_vector& bv,
                        std::string ol = default_output_label)
        : output_table<bool>(bv, ol) {}
    truth_output_table(const combo_tree& tr,
                       const truth_input_table& tti,
                       RandGen& rng);
    truth_output_table(const combo_tree& tr,
                       const ctruth_table& ctt,
                       RandGen& rng);
    
};

class truth_table : public table<truth_input_table, truth_output_table> {
    typedef table<truth_input_table, truth_output_table> super;
public:
    typedef ctruth_table CTable;

    truth_table(const super& t) : super(t) {}
    truth_table(std::istream& in, int pos = -1) : super(in, pos) {}
    truth_table(const std::string& file_name, int pos = -1) :
        super(file_name, pos) {}

    /// return the corresponding compressed truth table 
    CTable compress() const;
};

//////////////////
// contin table //
//////////////////

//shorthands used by class contin_input_table and contin_output_table
typedef std::vector<contin_t> contin_vector;
typedef contin_vector::iterator cv_it;
typedef contin_vector::const_iterator const_cv_it;
typedef std::vector<contin_vector> contin_matrix;
typedef contin_matrix::iterator cm_it;
typedef contin_matrix::const_iterator const_cm_it;

/*
  class contin_input_table
    matrix of randomly generated contin_t of sample_count rows and arity columns
*/
class contin_input_table : public input_table<contin_t>
{
    typedef input_table<contin_t> super;
public:
    // constructors
    contin_input_table() {}
    contin_input_table(const super& it) : super(it) {}
    contin_input_table(int sample_count, int arity, RandGen& rng,
                       double max_randvalue = 1.0, double min_randvalue = -1.0);
};

/*
  class contin_output_table
    contains sample_count evaluations obtained by evaluating t, a tree, over
    a RndNumTable cti.
    assumption : t has only contin inputs and output
*/
class contin_output_table : public output_table<contin_t>   //a column of results
{
public:
    typedef output_table<contin_t> super;

    //constructors
    contin_output_table(const std::string& ol = default_output_label)
        : super(ol) {}
    contin_output_table(const contin_vector& cv,
                        std::string ol = default_output_label) 
        : super(cv, ol) {}
    contin_output_table(const combo_tree& tr, const contin_input_table& cti,
                 RandGen& rng);
    template<typename Func>
    contin_output_table(const Func& f, const contin_input_table& cti) {
        foreach(const contin_vector& v, cti)
            push_back(f(v.begin(), v.end()));
    }

    //equality operator
    bool operator==(const contin_output_table& ct) const;
    bool operator!=(const contin_output_table& ct) const {
        return !operator==(ct);
    }
    // total of the absolute distance of each value
    contin_t abs_distance(const contin_output_table& other) const;
    // total of the squared error of each value
    contin_t sum_squared_error(const contin_output_table& other) const;
    // mean of the squared error of each value
    contin_t mean_squared_error(const contin_output_table& other) const;
    // sqrt of mean_squared_error
    contin_t root_mean_square_error(const contin_output_table& other) const;
};

struct contin_table : public table<contin_input_table, contin_output_table> {
    typedef table<contin_input_table, contin_output_table> super;
    contin_table(std::istream& in, int pos = -1) : super(in, pos) {}
    contin_table(const std::string& file_name, int pos = -1)
        : super(file_name, pos) {}
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
    //complete_truth_table
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
                bmap[_bool_arg[bai] + 1] = bool_to_vertex((i >> bai) % 2);
            for (const_cm_it si = cti.begin(); si != cti.end(); ++si) {
                int cai = 0; //contin arg index
                for (const_cv_it j = (*si).begin(); j != (*si).end(); ++j, ++cai)
                    bmap[_contin_arg[cai] + 1] = *j;
                vertex e = eval_throws_binding(rng, bmap, tr);
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

    binding_map bmap;
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
    //are enumerated in lexico order as for complete_truth_table
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
                    bmap[_contin_arg[cai] + 1] = *j;
                vertex e = eval_throws_binding(rng, bmap, tr);
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

    binding_map bmap;
};

/**
 * if the DSV data file has a header with labels
 */
std::vector<std::string> readInputLabels(const std::string& file, int pos = -1);

std::ifstream* open_data_file(const std::string& fileName);

template<typename T>
inline std::ostream& operator<<(std::ostream& out,
                                const input_table<T>& it)
{
    ostreamContainer(out, it.get_labels(), ",");
    foreach(const std::vector<T>& row, it) {
        ostreamContainer(out, row, ",");
        out << std::endl;
    }
    return out;
}

template<typename T>
inline std::ostream& operator<<(std::ostream& out,
                                const output_table<T>& ot)
{
    if(!ot.get_label().empty())
        out << ot.get_label() << std::endl;
    return ostreamContainer(out, ot, "\n");
}

inline std::ostream& operator<<(std::ostream& out,
                                const complete_truth_table& tt)
{
    return opencog::ostreamContainer(out, tt);
}

inline std::ostream& operator<<(std::ostream& out,
                                const mixed_table& mt)
{
    const std::vector<boost::variant<bool, contin_t> >& vt = mt.get_vt();
    for (unsigned i = 0; i != vt.size(); ++i)
        out << (boost::get<contin_t>(&vt[i]) ?
                boost::get<contin_t>(vt[i]) :
                boost::get<bool>(vt[i])) << " ";
    return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const mixed_action_table& mat)
{
    const std::vector<boost::variant<bool, contin_t> >& vt = mat.get_vt();
    for (unsigned i = 0; i != vt.size(); ++i)
        out << (boost::get<contin_t>(&vt[i]) ?
                boost::get<contin_t>(vt[i]) :
                boost::get<bool>(vt[i])) << " ";
    return out;
}

}} // ~namespaces combo opencog


// TODO see if we can put that under opencog combo
namespace boost
{
inline size_t hash_value(const opencog::combo::complete_truth_table& tt)
{
    return hash_range(tt.begin(), tt.end());
}
} //~namespace boost

#endif // _OPENCOG_TABLE_H
