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
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/tokenizer.hpp>

#include <opencog/util/RandGen.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/dorepeat.h>
#include <opencog/util/Counter.h>

#include "eval.h"
#include "vertex.h"
#include "common_def.h"

#define COEF_SAMPLE_COUNT 20.0 // involved in the formula that counts
                               // the number of trials needed to check
                               // a formula

namespace opencog { namespace combo {

using boost::variant;
using boost::adaptors::map_values;

///////////////////
// Generic table //
///////////////////

static const std::string default_input_label("i");

/// Contain a compressed table, for instance if the following table
/// is:
///
/// o,i1,i2
/// 1,1,0
/// 0,1,1
/// 1,1,0
/// 0,1,0
///
/// the compressed table is
///
/// o,i1,i2
/// {0:1,1:2},1,0
/// {0:1},1,1
///
/// that is the duplicated inputs are removed and the output is
/// replaced by a counter of the false ones and the true ones
/// respectively.
class CTable : public std::map<vertex_seq, std::map<vertex, unsigned> > {
public:
    typedef vertex_seq key_type;
    typedef std::map<vertex, unsigned> mapped_type;
    typedef std::map<key_type, mapped_type> super;

    std::string olabel;               // output label
    std::vector<std::string> ilabels; // list of input labels

    CTable(const std::string& _olabel, const std::vector<std::string>& _ilabels)
        : olabel(_olabel), ilabels(_ilabels) {}

    binding_map get_binding_map(const key_type& args) const {
        binding_map bmap;
        for(size_t i = 0; i < args.size(); ++i)
            bmap[i+1] = args[i];
        return bmap;
    }
};

/**
 * Matrix of vertexes.
 * Rows represent samples.
 * Columns represent input variables
 * Optionally a list of labels (input variable names)
 */
class ITable : public std::vector<vertex_seq> {
public:
    typedef std::vector<vertex_seq > super;
    ITable();
    ITable(const super& mat,
           std::vector<std::string> il = std::vector<std::string>());
    /**
     * generate an input table according to the signature tt.
     *
     * @param tt signature of the table to generate
     * @param nsamples sample size, if negative then the sample size is automatically determined
     * @param min_contin minimum contin value
     * @param max_contin maximum contin value
     */
    // min_contin and max_contin are used in case tt has contin inputs
    ITable(const type_tree& tt, RandGen& rng, int nsamples = -1,
           contin_t min_contin = -1.0, contin_t max_contin = 1.0);
    // set input labels
    void set_labels(std::vector<std::string> il);
    const std::vector<std::string>& get_labels() const;
    // like get_labels but filter accordingly to a container of arity_t
    template<typename F>
    std::vector<std::string> get_filtered_labels(const F& filter) {
        std::vector<std::string> res;
        foreach(arity_t a, filter)
            res.push_back(get_labels()[a]);
        return res;
    }
    // get binding map prior calling the combo evaluation
    binding_map get_binding_map(const vertex_seq& args) const {
        binding_map bmap;
        for(arity_t i = 0; i < (arity_t)args.size(); ++i)
            bmap[i+1] = args[i];
        return bmap;
    }
    arity_t get_arity() const {
        return super::front().size();
    }
    bool operator==(const ITable& rhs) const {
        return 
            static_cast<const super&>(*this) == static_cast<const super&>(rhs)
            && get_labels() == rhs.get_labels();
    }
    /// return a copy of the input table filtered according to a given
    /// container of arity_t
    template<typename F>
    ITable filter(const F& f) {
        ITable res;
        res.set_labels(get_filtered_labels(f));
        foreach(const typename super::value_type& row, *this) {
            vertex_seq new_row;
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
    /**
     * this function take an arity in input and returns in output the
     * number of samples that would be appropriate to check the semantics
     * of its associated tree.
     *
     * Note : could take the two trees to checking and according to their
     * arity structure, whatever, find an appropriate number.
     */
    unsigned sample_count(arity_t contin_arity)
    {
        if (contin_arity == 0)
            return 1;
        else return COEF_SAMPLE_COUNT*log(contin_arity + EXPONENTIAL);
    }

};
        
static const std::string default_output_label("output");
        
class OTable : public vertex_seq {
    typedef vertex_seq super;
public:
    typedef vertex value_type;

    OTable(const std::string& ol = default_output_label);
    OTable(const super& ot, const std::string& ol = default_output_label);
    OTable(const combo_tree& tr, const ITable& itable, RandGen& rng,
           const std::string& ol = default_output_label);
    OTable(const combo_tree& tr, const CTable& ctable, RandGen& rng,
           const std::string& ol = default_output_label);
    template<typename Func>
    OTable(const Func& f, const ITable& it,
           const std::string& ol = default_output_label)
        : label(ol) {
        foreach(const vertex_seq& vs, it)
            push_back(f(vs.begin(), vs.end()));        
    }

    void set_label(const std::string& ol);
    const std::string& get_label() const;
    bool operator==(const OTable& rhs) const;
    contin_t abs_distance(const OTable& ot) const;
    contin_t sum_squared_error(const OTable& ot) const;
    contin_t mean_squared_error(const OTable& ot) const;
    contin_t root_mean_square_error(const OTable& ot) const;
private:
    std::string label; // output label
};

struct Table {
    typedef vertex value_type;

    Table();
    Table(const combo_tree& tr, RandGen& rng, int nsamples = -1,
          contin_t min_contin = -1.0, contin_t max_contin = 1.0);
    size_t size() const { return itable.size(); }
    arity_t get_arity() const { return itable.get_arity(); }
    template<typename F> Table filter(const F& f) {
        Table res;
        res.itable = itable.filter(f);
        res.otable = otable;
        return res;
    }
    /// return the corresponding compressed table 
    CTable compress() const;

    type_tree tt;
    ITable itable;
    OTable otable;
};

////////////////////////
// Mutual Information //
////////////////////////

/**
 * Compute the entropy H(Y) of an output table. It assumes the data
 * are discretized.
 */
double OTEntropy(const OTable& ot);

/**
 * Given a feature set X1, ..., Xn provided a set of indices of the
 * column of an input table of type IT, and an output feature Y
 * provided by the output table of type OT, compute the mutual
 * information
 *
 * MI(X1, ..., Xn; Y)
 *
 * @note only works for discrete data set.
 */
template<typename FeatureSet>
double mutualInformation(const ITable& it, const OTable& ot, const FeatureSet& fs) {
    // the following mapping is used to keep track of the number
    // of inputs a given setting. For instance X1=false, X2=true,
    // X3=true is one possible setting. It is then used to compute
    // H(Y, X1, ..., Xn) and H(X1, ..., Xn)
    typedef Counter<vertex_seq, unsigned> VSCounter;
    VSCounter ic, // for H(X1, ..., Xn)
        ioc; // for H(Y, X1, ..., Xn)
    ITable::const_iterator i_it = it.begin();
    OTable::const_iterator o_it = ot.begin();
    for(; i_it != it.end(); ++i_it, ++o_it) {
        vertex_seq ic_vec;
        foreach(const typename FeatureSet::value_type& idx, fs)
            ic_vec.push_back((*i_it)[idx]);
        ++ic[ic_vec];
        vertex_seq ioc_vec(ic_vec);
        ioc_vec.push_back(*o_it);
        ++ioc[ioc_vec];
    }
    // Compute the probability distributions
    std::vector<double> ip(ic.size()), iop(ioc.size());
    double total = it.size();
    auto div_total = [&](unsigned c) { return c/total; };
    transform(ic | map_values, ip.begin(), div_total);
    transform(ioc | map_values, iop.begin(), div_total);
    // Compute the entropies
    return entropy(ip) + OTEntropy(ot) - entropy(iop);
}

// like above but taking a table in argument instead of input and output tables
template<typename FeatureSet>
double mutualInformation(const Table& table, const FeatureSet& fs) {
    return mutualInformation(table.itable, table.otable, fs);
}

/**
 * Like above but uses a compressed table instead of input and output
 * table. It assumes the output is boolean. The CTable cannot be
 * passed as const because the use of the operator[] may modify it's
 * content (by adding default value on missing keys).
 */
template<typename FeatureSet>
double mutualInformation(CTable& ctable, const FeatureSet& fs) {
    // the following mapping is used to keep track of the number
    // of inputs a given setting. For instance X1=false, X2=true,
    // X3=true is one possible setting. It is then used to compute
    // H(Y, X1, ..., Xn) and H(X1, ..., Xn)
    typedef Counter<vertex_seq, unsigned> VSCounter;
    VSCounter ic, // for H(X1, ..., Xn)
        ioc; // for H(Y, X1, ..., Xn)
    unsigned oc = 0; // for H(Y)
    double total = 0;
    foreach(auto& row, ctable) {
        unsigned falses = row.second[id::logical_false];
        unsigned trues = row.second[id::logical_true];
        unsigned row_total = falses + trues;
        // update ic
        vertex_seq vec;
        foreach(unsigned idx, fs)
            vec.push_back(row.first[idx]);
        ic[vec] += row_total;
        // update ioc
        if(falses > 0) {
            vec.push_back(false);
            ioc[vec] += falses;
            vec.pop_back();
        }
        if(trues > 0) {
            vec.push_back(true);
            ioc[vec] += trues;
        }
        // update oc
        oc += trues;
        // update total
        total += row_total;
    }
    // Compute the probability distributions
    std::vector<double> ip(ic.size()), iop(ioc.size());
    auto div_total = [&](unsigned c) { return c/total; };
    transform(ic | map_values, ip.begin(), div_total);
    transform(ioc | map_values, iop.begin(), div_total);
    // Compute the entropies
    return entropy(ip) + binaryEntropy(oc/total) - entropy(iop);
}

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
 * check if the data file has a header. That is whether the first row
 * starts with a sequence of output and input labels
 */
bool has_header(const std::string& dataFileName);

/**
 * Check the token, if it is "0" or "1" then it is boolean, otherwise
 * it is contin. It is not 100% reliable of course and should be
 * improved.
 */
type_node infer_type_from_token(const std::string& token);

/**
 * take a row in input as a pair {inputs, output} and return the type
 * tree corresponding to the function mapping inputs to output. If the
 * inference fails then it returns a type_tree with
 * id::ill_formed_type as root.
 */
type_tree infer_row_type_tree(std::pair<std::vector<std::string>,
                                        std::string>& row);

/**
 * Infer the type_tree of the function given underlying the data file
 */
type_tree infer_data_type_tree(const std::string& dataFileName, int pos = 0);

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
std::pair<std::vector<T>, T> tokenizeRowIO(std::string& line, int pos = 0) {
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
 * Fill an input table and output table given a DSV
 * (delimiter-seperated values) file format, where delimiters are ',',
 * ' ' or '\t'.
 *
 * It is assumed that each row have the same number of columns, if not
 * an assert is raised.
 *
 * pos specifies the position of the output, if -1 it is the last
 * position. The default position is 0, the first column.
 */
std::istream& istreamTable(std::istream& in, ITable& it, OTable& ot,
                           bool has_header, const type_tree& tt, int pos = 0);
/**
 * like above but take an string (file name) instead of istream. If
 * the file name is not correct then an OC_ASSERT is raised.
 */
void istreamTable(const std::string& file_name,
                  ITable& it, OTable& ot, int pos = 0);
/**
 * like above but return an object Table. 
 */
Table istreamTable(const std::string& file_name, int pos = 0);

//////////////////
// ostreamTable //
//////////////////

// output the header of a data table in CSV format.
std::ostream& ostreamTableHeader(std::ostream& out,
                                 const ITable& it, const OTable& ot);

// output a data table in CSV format. Boolean values are output in
// binary form (0 for false, 1 for true)
std::ostream& ostreamTable(std::ostream& out,
                           const ITable& it, const OTable& ot);
// like above but take a table instead of an input and output table
std::ostream& ostreamTable(std::ostream& out, const Table& table);

// like above but takes the file name where to write the table
void ostreamTable(const std::string& file_name,
                  const ITable& it, const OTable& ot);
// like above but take a table instead of a input and output table
void ostreamTable(const std::string& file_name, const Table& table);

// like ostreamTableHeader but on a compressed table
std::ostream& ostreamCTableHeader(std::ostream& out, const CTable& ct);

// output a compress table in pseudo CSV format
std::ostream& ostreamCTable(std::ostream& out, const CTable& ct);
        
/**
 * template to subsample input and output tables, after subsampling
 * the table have size min(nsamples, *table.size())
 */
void subsampleTable(ITable& it, OTable& ot,
                    unsigned int nsamples, RandGen& rng);

/**
 * like above but subsample only the input table
 */
void subsampleTable(ITable& it, unsigned int nsamples, RandGen& rng);

/////////////////
// Truth table //
/////////////////

//////////////////////////////
// probably soon deprecated //
//////////////////////////////
 
// shorthands used by class contin_input_table and contin_output_table
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
 * +-----------------------+--+--+
 * |Output                 |#1|#2|
 * +-----------------------+--+--+
 * |complete_truth_table[0]|F |F |
 * +-----------------------+--+--+
 * |complete_truth_table[1]|T |F |
 * +-----------------------+--+--+
 * |complete_truth_table[2]|F |T |
 * +-----------------------+--+--+
 * |complete_truth_table[3]|T |T |
 * +-----------------------+--+--+
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

//////////////////
// contin table //
//////////////////

//////////////////////////////
// probably soon deprecated //
//////////////////////////////
 
//shorthands used by class contin_input_table and contin_output_table
typedef std::vector<contin_t> contin_vector;
typedef contin_vector::iterator cv_it;
typedef contin_vector::const_iterator const_cv_it;
typedef std::vector<contin_vector> contin_matrix;
typedef contin_matrix::iterator cm_it;
typedef contin_matrix::const_iterator const_cm_it;


/**
 * if the DSV data file has a header with labels
 */
std::vector<std::string> readInputLabels(const std::string& file, int pos = 0);

std::ifstream* open_data_file(const std::string& fileName);

inline std::ostream& operator<<(std::ostream& out, const ITable& it)
{
    ostreamlnContainer(out, it.get_labels(), ",");
    foreach(const vertex_seq& row, it)
        ostreamlnContainer(out, row, ",");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const OTable& ot)
{
    if(!ot.get_label().empty())
        out << ot.get_label() << std::endl;
    return ostreamContainer(out, ot, "\n");
}

inline std::ostream& operator<<(std::ostream& out,
                                const complete_truth_table& tt)
{
    return ostreamContainer(out, tt);
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
