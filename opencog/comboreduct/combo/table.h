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

#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/transform.hpp>

#include <opencog/util/Counter.h>
#include <opencog/util/algorithm.h>

#include "eval.h"   /* Needed for binding map, and then obsolete */
#include "vertex.h"
#include "common_def.h"

#define COEF_SAMPLE_COUNT 20.0 // involved in the formula that counts
                               // the number of trials needed to check
                               // a formula

namespace opencog { namespace combo {

///////////////////
// Generic table //
///////////////////

/// CTable is a "compressed" table.  Compression is done by removing
/// duplicated inputs, and the output column is replaced by a counter
/// of the duplicated outputs.  That is, the output column is of the
/// form {v1:c1, v2:c2, ...} where c1 is the number of times value v1
/// was seen in the output, c2 the number of times v2 was observed, etc.
///
/// For example, if one has the following table:
///
///   output, input1, input2
///   1,1,0
///   0,1,1
///   1,1,0
///   0,1,0
///
/// Then the compressed table is
///
///   output, input1, input2
///   {0:1,1:2},1,0
///   {0:1},1,1
///
/// Most scoring functions work on CTable, as it avoids re-evaluating a
/// combo program on duplicated inputs.
//
class CTable : public std::map<vertex_seq, Counter<vertex, unsigned>>
{
public:
    typedef vertex_seq key_type;
    typedef Counter<vertex, unsigned> counter_t;
    typedef std::map<key_type, counter_t> super;
    typedef typename super::value_type value_type;
    typedef std::vector<std::string> string_seq;

    // Definition is delayed until after Table, as it uses Table.
    template<typename Func>
    CTable(const Func& func, arity_t arity, int nsamples = -1);

    CTable(const std::string& _olabel, const string_seq& _ilabels)
        : olabel(_olabel), ilabels(_ilabels) {}

    arity_t get_arity() const { return ilabels.size(); }

    // Return the total number of observations (should be equal to the
    // size of the corresponding uncompressed table)
    unsigned uncompressed_size() const {
        unsigned res = 0;
        foreach(const value_type& v, *this) {
            res += v.second.total_count();
        }
        return res;
    }

    template<typename F>
    CTable filtered(const F& filter) const {
        typedef type_tree::iterator pre_it;
        typedef type_tree::sibling_iterator sib_it;

        // Filter the labels
        CTable res(olabel, seq_filtered(ilabels, filter));

        // Filter the rows
        foreach(const CTable::value_type v, *this)
            res[seq_filtered(v.first, filter)] += v.second;

        // Filter the type tree
        // copy head
        pre_it head_src = tt.begin();
        OC_ASSERT(*head_src == id::lambda_type);
        OC_ASSERT((int)tt.number_of_children(head_src) == get_arity() + 1);
        pre_it head_dst = res.tt.set_head(*head_src);
        // copy filtered input types
        sib_it sib_src = head_src.begin();
        arity_t a_pre = 0;
        foreach(arity_t a, filter) {
            std::advance(sib_src, a - a_pre);
            a_pre = a;
            res.tt.replace(res.tt.append_child(head_dst), sib_src);
        }
        // copy output type
        res.tt.replace(res.tt.append_child(head_dst), head_src.last_child());

        // return the filtered CTable
        return res;
    }

    template<typename F, typename Seq>
    Seq filtered_preverse_idxs(const F& filter, const Seq& seq) const {
        Seq res;
        auto it = filter.cbegin();
        for (unsigned i = 0; i < seq.size(); ++i) {
            if (it != filter.cend() && (typename F::value_type)i == *it) {
                res.push_back(seq[i]);
                ++it;
            } else
                res.push_back(id::null_vertex);
        }
        return seq;
    }

    /**
     * Like filtered but preserve the indices on the columns,
     * techincally it replaces all input values filtered out by
     * id::null_vertex.
     */
    template<typename F>
    CTable filtered_preverse_idxs(const F& filter) const {
        typedef type_tree::iterator pre_it;
        typedef type_tree::sibling_iterator sib_it;

        // Set new CTable
        CTable res(olabel, ilabels);

        // Filter the rows (replace filtered out values by id::null_vertex)
        foreach(const CTable::value_type v, *this)
            res[filtered_preverse_idxs(filter, v.first)] += v.second;

        // return the filtered CTable
        return res;
    }

    string_seq get_labels() const;

// protected:
    type_tree tt;
    std::string olabel;               // output label
    string_seq ilabels; // list of input labels

};



/**
 * Input table of vertexes.
 * Rows represent data samples.
 * Columns represent input variables.
 * Optionally holds a list of column labels (input variable names)
 */
class ITable : public std::vector<vertex_seq>
{
public:
    typedef std::vector<vertex_seq> super;
    typedef std::vector<std::string> string_seq;
    typedef std::vector<type_node> type_seq;
    ITable();
    ITable(const super& mat, string_seq il = string_seq());
    /**
     * generate an input table according to the signature tt.
     *
     * @param tt signature of the table to generate.
     * @param nsamples sample size, if negative then the sample
              size is automatically determined.
     * @param min_contin minimum contin value.
     * @param max_contin maximum contin value.
     *
     * It onyl works for contin-boolean signatures
     */
    // min_contin and max_contin are used in case tt has contin inputs
    ITable(const type_tree& tt, int nsamples = -1,
           contin_t min_contin = -1.0, contin_t max_contin = 1.0);

    arity_t get_arity() const {
        return super::front().size();
    }

    bool operator==(const ITable& rhs) const;

    // set input labels
    void set_labels(const string_seq&);
    const string_seq& get_labels() const;

    void set_types(const type_seq&);
    const type_seq& get_types() const;
    type_node get_type(const std::string&) const;

    /**
     * Insert a column 'col', named 'clab', after position 'off'
     * If off is negative, then the insert is after the last column.
     * TODO: we really should use iterators here, not column numbers.
     */
    void insert_col(const std::string& clab,
                    const vertex_seq& col,
                    int off = -1);
    /**
     * Delete the named feature from the input table.
     * If the feature is the empty string, then column zero is deleted.
     * The returned value is the name of the column.
     */
    std::string delete_column(const std::string& feature);
    void delete_columns(const string_seq& ignore_features);

    /**
     * Get the column, given its offset or label
     */
    vertex_seq get_column_data(const std::string& name) const;
    vertex_seq get_column_data(int offset) const;

    /// return a copy of the input table filtered according to a given
    /// container of arity_t. Each value of that container corresponds
    /// to the column index of the ITable (starting from 0).
    template<typename F>
    ITable filtered(const F& filter) const
    {
        ITable res;

        // filter labels
        res.set_labels(seq_filtered(get_labels(), filter));

        // filter types
        res.set_types(seq_filtered(get_types(), filter));

        // filter content
        foreach(const value_type& row, *this) {
            vertex_seq new_row;
            foreach(arity_t a, filter)
                new_row.push_back(row[a]);
            res.push_back(new_row);
        }
        return res;
    }

protected:
    mutable string_seq labels; // list of input labels
    mutable type_seq types;    // list of types of the columns

private:
    string_seq get_default_labels() const;
    int get_column_offset(const std::string& col_name) const;

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

/**
 * Output table of vertexes.
 * Rows represent dependent data samples.
 * There is only one column: a single output value for each row.
 * Optionally holds a column label (output variable names)
 */
class OTable : public vertex_seq
{
    typedef vertex_seq super;
public:
    typedef vertex value_type;

    OTable(const std::string& ol = default_output_label);
    OTable(const super& ot, const std::string& ol = default_output_label);

    /// Construct the OTable by evaluating the combo tree @tr for each
    /// row in the input ITable.
    OTable(const combo_tree& tr, const ITable& itable,
           const std::string& ol = default_output_label);

    /// Construct the OTable by evaluating the combo tree @tr for each
    /// row in the input CTable.
    OTable(const combo_tree& tr, const CTable& ctable,
           const std::string& ol = default_output_label);

    template<typename Func>
    OTable(const Func& f, const ITable& it,
           const std::string& ol = default_output_label)
        : label(ol)
    {
        foreach(const vertex_seq& vs, it)
            push_back(f(vs.begin(), vs.end()));
    }

    void set_label(const std::string&);
    const std::string& get_label() const;
    void set_type(type_node);
    type_node get_type() const;
    bool operator==(const OTable& rhs) const;
    contin_t abs_distance(const OTable&) const;
    contin_t sum_squared_error(const OTable&) const;
    contin_t mean_squared_error(const OTable&) const;
    contin_t root_mean_square_error(const OTable&) const;

    vertex get_enum_vertex(const std::string& token);

protected:
    std::string label; // output label
    type_node type;
};

/**
 * Typed data table.
 * The table consists of an ITable of inputs (independent variables),
 * an OTable holding the output (the dependent variable), and a type
 * tree identifiying the types of the inputs and outputs.
 */
struct Table
{
    typedef std::vector<std::string> string_seq;
    typedef vertex value_type;

    Table();

    Table(const OTable& otable_, const ITable& itable_, const type_tree& tt_);

    template<typename Func>
    Table(const Func& func, arity_t a, int nsamples = -1) :
        tt(gen_signature(type_node_of<bool>(),
                         type_node_of<bool>(), a)),
        itable(tt), otable(func, itable) {}

    Table(const combo_tree& tr, int nsamples = -1,
          contin_t min_contin = -1.0, contin_t max_contin = 1.0);
    size_t size() const { return itable.size(); }
    arity_t get_arity() const { return itable.get_arity(); }
    const type_tree& get_signature() const { return tt; }
    string_seq get_labels() const;
    const std::string& get_target() const { return otable.get_label(); }

    // Filter according to a container of arity_t. Each value of that
    // container corresponds to the column index of the ITable
    // (starting from 0).
    template<typename F> Table filtered(const F& f) const {
        Table res;

        // filter input table
        res.itable = itable.filtered(f);

        // set output table
        res.otable = otable;

        // set type tree
        type_tree::iterator head = res.tt.set_head(id::lambda_type);
        foreach(type_node tn, res.itable.get_types())
            res.tt.append_child(head, tn);
        res.tt.append_child(head, otable.get_type());
            
        return res;
    }

    /// return the corresponding compressed table
    CTable compressed() const;

    type_tree tt;
    ITable itable;
    OTable otable;
};

template<typename Func>
CTable::CTable(const Func& func, arity_t arity, int nsamples) {
    Table table(func, arity, nsamples);
    *this = table.compressed();
}


////////////////////////
// Mutual Information //
////////////////////////

/**
 * Compute the joint entropy H(Y) of an output table. It assumes the data
 * are discretized. (?)
 */
double OTEntropy(const OTable& ot);

/**
 * Compute the mutual information between a set of independent features
 * X_1, ... X_n and a taget feature Y.
 *
 * The target (output) featuer Y is provided in the output table OTable,
 * whereas the input features are specified as a set of indexes giving
 * columns in the input table ITable.
 *
 * The mutual information
 *
 *   MI(Y; X1, ..., Xn)
 *
 * is computed as
 *
 *   MI(Y;X1, ..., Xn) = H(X1, ..., Xn) + H(Y) - H(X1, ..., Xn, Y)
 *
 * where
 *   H(...) are the joint entropies.
 *
 * @note only works for discrete data set.
 */
template<typename FeatureSet>
double mutualInformation(const ITable& it, const OTable& ot, const FeatureSet& fs)
{
    // The following mapping is used to keep track of the number
    // of inputs a given setting. For instance, X1=false, X2=true,
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

    // Compute the joint entropies
    return entropy(ip) + OTEntropy(ot) - entropy(iop);
}

// Like the above, but taking a table in argument instead of
// input and output tables
template<typename FeatureSet>
double mutualInformation(const Table& table, const FeatureSet& fs)
{
    return mutualInformation(table.itable, table.otable, fs);
}

/**
 * Like above but uses a compressed table instead of input and output
 * table. It assumes the output is boolean. The CTable cannot be
 * passed as const because the use of the operator[] may modify it's
 * content (by adding default value on missing keys).
 */
template<typename FeatureSet>
double mutualInformation(const CTable& ctable, const FeatureSet& fs)
{
    // the following mapping is used to keep track of the number
    // of inputs a given setting. For instance X1=false, X2=true,
    // X3=true is one possible setting. It is then used to compute
    // H(Y, X1, ..., Xn) and H(X1, ..., Xn)
    typedef Counter<vertex_seq, unsigned> VSCounter;
    VSCounter ic, // for H(X1, ..., Xn)
        ioc; // for H(Y, X1, ..., Xn)
    unsigned oc = 0; // for H(Y)
    double total = 0;
    foreach(const auto& row, ctable) {
        unsigned falses = row.second.get(id::logical_false);
        unsigned trues = row.second.get(id::logical_true);
        unsigned row_total = falses + trues;
        // update ic
        vertex_seq vec;
        foreach(unsigned idx, fs)
            vec.push_back(row.first[idx]);
        ic[vec] += row_total;
        // update ioc
        if (falses > 0) {
            vec.push_back(id::logical_false);
            ioc[vec] += falses;
            vec.pop_back();
        }
        if (trues > 0) {
            vec.push_back(id::logical_true);
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

/**
 * template to subsample input and output tables, after subsampling
 * the table have size min(nsamples, *table.size())
 */
void subsampleTable(ITable& it, OTable& ot, unsigned nsamples);

/**
 * Like above on Table instead of ITable and OTable
 */
void subsampleTable(Table& table, unsigned nsamples);

/**
 * like above but subsample only the input table
 */
void subsampleTable(ITable& it, unsigned nsamples);

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
 * |Output                 |$1|$2|
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

    complete_truth_table() {}
    template<typename It>
    complete_truth_table(It from, It to) : super(from, to) {}
    template<typename T>
    complete_truth_table(const tree<T>& tr, arity_t arity)
        : super(pow2(arity)), _arity(arity)
    {
        populate(tr);
    }
    template<typename T>
    complete_truth_table(const tree<T>& tr)
    {
        _arity = arity(tr);
        this->resize(pow2(_arity));
        populate(tr);
    }

    template<typename Func>
    complete_truth_table(const Func& f, arity_t arity)
        : super(pow2(arity)), _arity(arity) {
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
    void populate(const tree<T>& tr)
    {
        bmap.resize(_arity);
        iterator it = begin();
        for (int i = 0; it != end(); ++i, ++it) {
            for (int j = 0; j < _arity; ++j)
                bmap[j] = bool_to_vertex((i >> j) % 2);
            *it = eval_binding(bmap, tr) == id::logical_true;
        }
    }
    arity_t _arity;
    mutable vertex_seq bmap;
};

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
