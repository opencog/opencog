/** table.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Authors: Nil Geisweiller <ngeiswei@gmail.com>
 *          Linas Vepstas <linasvepstas@gmail.com>
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
#include "table.h"

#include <iomanip>
#include <ctype.h>
#include <stdlib.h>

#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/adjacent_find.hpp>
#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <opencog/util/dorepeat.h>
#include <opencog/util/Logger.h>

#include "../combo/ann.h"
#include "../combo/simple_nn.h"
#include "../combo/convert_ann_combo.h"

namespace opencog { namespace combo {

using namespace std;
using namespace boost;
using namespace boost::adaptors;

string table_fmt_builtin_to_str(const builtin& b)
{
    stringstream ss;
    if (is_boolean(b))
        ss << builtin_to_bool(b);
    else
        ss << b;
    return ss.str();
}
string table_fmt_vertex_to_str(const vertex& v)
{
    stringstream ss;
    if (is_boolean(v))
        ss << vertex_to_bool(v);
    else
        ss << v;
    return ss.str();
}
        
// -------------------------------------------------------

ITable::ITable() {}

ITable::ITable(const vector<type_node>& ts, const vector<string>& il)
    : types(ts), labels(il) {}

ITable::ITable(const ITable::super& mat, const vector<string>& il)
    : super(mat), labels(il) {}

ITable::ITable(const type_tree& tt, int nsamples,
               contin_t min_contin, contin_t max_contin)
{
    arity_t barity = boolean_arity(tt), carity = contin_arity(tt);

    if (nsamples < 0)
        nsamples = std::max(pow2(barity), sample_count(carity));

    // in that case the boolean inputs are not picked randomly but
    // instead are enumerated
    bool comp_tt = nsamples == (int)pow2(barity);

    // Populate the matrix.
    auto root = tt.begin();
    for (int i = 0; i < nsamples; ++i) {
        size_t bidx = 0;        // counter used to enumerate all
                                // booleans
        vertex_seq vs;
        foreach (type_node n, make_pair(root.begin(), root.last_child()))
            if (n == id::boolean_type)
                vs.push_back(bool_to_vertex(comp_tt?
                                            i & (1 << bidx++)
                                            : randGen().randint(2)));
            else if (n == id::contin_type)
                vs.push_back((max_contin - min_contin)
                             * randGen().randdouble() + min_contin);
            else if (n == id::enum_type)
                vs.push_back(enum_t::get_random_enum());
            else if (n == id::unknown_type)
                vs.push_back(vertex()); // push default vertex
            else
                OC_ASSERT(false, "Not implemented yet");

        // input vector
        push_back(vs);
    }
}

bool ITable::operator==(const ITable& rhs) const
{
    // return
    bool super_eq = static_cast<const super&>(*this) == static_cast<const super&>(rhs);
    bool labels_eq = get_labels() == rhs.get_labels();
    bool types_eq = get_types() == rhs.get_types();
    return super_eq && labels_eq && types_eq;
}

// -------------------------------------------------------

void ITable::set_labels(const vector<string>& il)
{
    labels = il;
}

static const std::string default_input_label("i");

vector<string> ITable::get_default_labels() const
{
    string_seq res;
    for (arity_t i = 1; i <= get_arity(); ++i)
        res.push_back(default_input_label
                      + boost::lexical_cast<std::string>(i));
    return res;
}


const vector<string>& ITable::get_labels() const
{
    if (labels.empty() and !super::empty()) // return default labels
        labels = get_default_labels();
    return labels;
}

void ITable::set_types(const vector<type_node>& il)
{
    types = il;
}

const vector<type_node>& ITable::get_types() const
{
    if (types.empty() and !super::empty()) {
        arity_t arity = get_arity();
        types.resize(arity);
        for (arity_t i=0; i<arity; i++) {
            types[i] = id::unknown_type;
        }
    }
    return types;
}

type_node ITable::get_type(const string& name) const
{
    if (types.empty())
        return id::unknown_type;

    int off = get_column_offset(name);
    if (-1 == off)
        return id::unknown_type;

    return types[off];
}

// -------------------------------------------------------

void ITable::insert_col(const std::string& clab,
                        const vertex_seq& col,
                        int off)
{
    // Infer the column type
    // If it exists use the second row, just in case the first holds labels...
    unsigned idx = col.size() > 1 ? 1 : 0;
    type_tree col_tt = get_type_tree(col[idx]);
    type_node col_type = get_type_node(col_tt);
    types.insert(off >= 0 ? types.begin() + off : types.end(), col_type);

    // Insert label
    labels.insert(off >= 0 ? labels.begin() + off : labels.end(), clab);

    // Insert values
    if (empty()) {
        OC_ASSERT(off < 0);
        for (const auto& v : col)
            push_back({v});
        return;
    }

    OC_ASSERT (col.size() == size(), "Incorrect column length!");
    for (unsigned i = 0; i < col.size(); i++) {
        auto& row = (*this)[i];

        // convert row into vertex_seq
        vertex_seq vs;
        for (unsigned j = 0; j < row.size(); ++j)
            vs.push_back(row.get_at<vertex>(j));
        row = vs;

        // insert the value from col at off
        row.insert_at(off, col[i]);
    }
}

int ITable::get_column_offset(const std::string& name) const
{
    // If the name is empty, get column zero.
    if (name.empty())
        return 0;

    // If the name is numeric, then assume its a column number
    // starting at column 1 for the leftmost column.
    // i.e. subtract one to get number.
    if (isdigit(name.c_str()[0]))
        return atoi(name.c_str()) - 1;

    auto pos = std::find(labels.begin(), labels.end(), name);
    if (pos == labels.end())
        return -1;
    return distance(labels.begin(), pos);
}

vertex_seq ITable::get_column_data(int offset) const
{
    // @todo it outputs vertex_seq, it's not very general
    
    vertex_seq col;

    if (-1 == offset)
        return col;

    get_at_visitor<vertex> gvav(offset);
    auto agva = boost::apply_visitor(gvav);
    for (const auto& row : *this) {
        col.push_back(agva(row.get_variant()));
    }
    return col;
}

vertex_seq ITable::get_column_data(const std::string& name) const
{
    return get_column_data(get_column_offset(name));
}

string ITable::delete_column(const string& name)
{
    int off = get_column_offset(name);
    if (-1 == off)
        return string();

    // Delete the column
    for (multi_type_seq& row : *this)
        row.erase_at(off);

    // Delete the label as well.
    string rv;
    if (!labels.empty()) {
        rv = *(labels.begin() + off);
        labels.erase(labels.begin() + off);
    }

    if (!types.empty())
        types.erase(types.begin() + off);

    return rv;
}


void ITable::delete_columns(const vector<string>& ignore_features)
{
    for (const string& feat : ignore_features)
        delete_column(feat);
}

// -------------------------------------------------------

OTable::OTable(const string& ol)
    : label(ol), type(id::unknown_type) {}

OTable::OTable(const super& ot, const string& ol)
    : super(ot), label(ol)
{
    // Be sure to set the column type as well ... 
    type = get_type_node(get_type_tree((*this)[0]));
}

OTable::OTable(const combo_tree& tr, const ITable& itable, const string& ol)
    : label(ol)
{
    OC_ASSERT(!tr.empty());
    if (is_ann_type(*tr.begin())) {
        // we treat ANN differently because they must be decoded
        // before being evaluated. Also note that if there are memory
        // neurones then the state of the network is evolving at each
        // input, so the order within itable does matter
        ann net = tree_transform().decodify_tree(tr);
        int depth = net.feedforward_depth();
        for (const multi_type_seq& vv : itable) {
            contin_seq tmp = vv.get_seq<contin_t>();
            tmp.push_back(1.0); // net uses that in case the function
                                // to learn needs some kind of offset
            net.load_inputs(tmp);
            dorepeat(depth)
                net.propagate();
            push_back(net.outputs[0]->activation);
        }
    } else {
        interpreter_visitor iv(tr);
        auto ai = boost::apply_visitor(iv);
        for (const multi_type_seq& vs : itable)
            push_back(ai(vs.get_variant()));
    }

    // Be sure to set the column type as well ... 
    type = get_type_node(get_type_tree((*this)[0]));
}

OTable::OTable(const combo_tree& tr, const CTable& ctable, const string& ol)
    : label(ol)
{
    arity_set as = get_argument_abs_idx_set(tr);
    interpreter_visitor iv(tr);
    auto ai = boost::apply_visitor(iv);
    for_each(ctable | map_keys, [&](const multi_type_seq& mts) {
            this->push_back(ai(mts.get_variant()));
        });

    // Be sure to set the column type as well ... 
    type = get_type_node(get_type_tree((*this)[0]));
}

void OTable::set_label(const string& ol)
{
    if (ol.empty())
        label = default_output_label;
    else
        label = ol;
}

const string& OTable::get_label() const
{
    return label;
}

void OTable::set_type(type_node t)
{
    type = t;
}

type_node OTable::get_type() const
{
    return type;
}

// -------------------------------------------------------

bool OTable::operator==(const OTable& rhs) const
{
    const static contin_t epsilon = 1e-12;
    for (auto lit = begin(), rit = rhs.begin(); lit != end(); ++lit, ++rit) {
        if (is_contin(*lit) && is_contin(*rit)) {
            if (!isApproxEq(get_contin(*lit), get_contin(*rit), epsilon))
                return false;
        }
        else if (*lit != *rit)
            return false;
    }
    return rhs.get_label() == label;
}

// XXX TODO replace this by the util p_norm function.
contin_t OTable::abs_distance(const OTable& ot) const
{
    OC_ASSERT(ot.size() == size());
    contin_t res = 0;
    for(const_iterator x = begin(), y = ot.begin(); x != end();)
        res += fabs(get_contin(*(x++)) - get_contin(*(y++)));
    return res;
}

// XXX TODO replace this by the util p_norm function.
contin_t OTable::sum_squared_error(const OTable& ot) const
{
    OC_ASSERT(ot.size() == size());
    contin_t res = 0;
    for(const_iterator x = begin(), y = ot.begin(); x != end();)
        res += sq(get_contin(*(x++)) - get_contin(*(y++)));
    return res;
}

contin_t OTable::mean_squared_error(const OTable& ot) const
{
    OC_ASSERT(ot.size() == size() && size() > 0);
    return sum_squared_error(ot) / ot.size();
}

contin_t OTable::root_mean_square_error(const OTable& ot) const
{
    OC_ASSERT(ot.size() == size() && size() > 0);
    return sqrt(mean_squared_error(ot));
}

// -------------------------------------------------------

Table::Table() : target_pos(0) {}

Table::Table(const OTable& otable_, const ITable& itable_)
    : itable(itable_), otable(otable_), target_pos(0) {}

Table::Table(const combo_tree& tr, int nsamples,
             contin_t min_contin, contin_t max_contin) :
    itable(infer_type_tree(tr), nsamples, min_contin, max_contin),
    otable(tr, itable), target_pos(0) {}

vector<string> Table::get_labels() const
{
    vector<string> labels = itable.get_labels();
    labels.insert(labels.begin(), otable.get_label());
    return labels;
}

// -------------------------------------------------------

CTable Table::compressed() const
{
    // Logger
    logger().debug("Compress the dataset, current size is %d", itable.size());
    // ~Logger

    CTable res(otable.get_label(), itable.get_labels(), get_signature());

    ITable::const_iterator in_it = itable.begin();
    OTable::const_iterator out_it = otable.begin();
    for(; in_it != itable.end(); ++in_it, ++out_it)
        ++res[*in_it][*out_it];

    logger().debug("Size of the compressed dataset is %d", res.size());

    return res;
}

// -------------------------------------------------------

std::istream& istreamRawITable(std::istream& in, ITable& tab,
                               const std::vector<unsigned>& ignored_indices)
    throw(std::exception, AssertionException);
std::vector<std::string> get_header(const std::string& input_file);

/**
 * Get indices (aka positions or offsets) of a list of labels given a header
 */
vector<unsigned> get_indices(const vector<string>& labels,
                             const vector<string>& header) {
    vector<unsigned> res;
    for (unsigned i = 0; i < header.size(); ++i)
        if (boost::find(labels, header[i]) != labels.end())
            res.push_back(i);
    return res;
}
unsigned get_index(const string& label, const vector<string>& header) {
    return distance(header.begin(), boost::find(header, label));
}

void Table::add_features_from_file(const string& input_file,
                                   vector<string> features)
{
    // consider only the features not already present
    const vector<string>& labels = itable.get_labels();
    for (const string& f : labels) {
        auto it = boost::find(features, f);
        if (it != features.end())
            features.erase(it);
    }

    // If no feature to force, there is nothing to do
    if (!features.empty()) {
        // header of the DSV file
        vector<string> full_header = get_header(input_file);

        // [0, header.size())
        vector<unsigned> full_header_pos = get_indices(full_header, full_header);
        // indices of table's features relative to full_header
        vector<unsigned> header_pos =  get_indices(labels, full_header);
        // indices of features to insert relative to full_header
        vector<unsigned> features_pos = get_indices(features, full_header);
        // target position relative to full_header
        int full_target_pos = get_index(otable.get_label(), full_header);

        // Get the complement of features_pos
        vector<unsigned> features_pos_comp;
        boost::set_difference(full_header_pos, features_pos,
                              back_inserter(features_pos_comp));

        // load the table with the features to insert with types
        // string that way the content is unchanged (convenient when
        // the data contains stuff that loadITable does not know how
        // to interpret).
        ITable features_table;
        ifstream in(input_file.c_str());
        istreamRawITable(in, features_table, features_pos_comp);

        // set the first row as header
        auto first_row_it = features_table.begin();
        vector<string> features_labels = first_row_it->get_seq<string>();
        features_table.set_labels(features_labels);
        features_table.erase(first_row_it);

        // Insert the forced features in the right order. We want to keep
        // the features in order because that is likely what the user
        // expects.
        // TODO UPDATE THE TYPE TREE
        
        // insert missing columns from features_itable to itable
        for (auto lit = features_pos.cbegin(), rit = header_pos.cbegin();
             lit != features_pos.cend(); ++lit) {
            int lpos = distance(features_pos.cbegin(), lit);
            vertex_seq cd = features_table.get_column_data(lpos);
            string cl = features_labels[lpos];
            while(rit != header_pos.cend() && *lit > *rit) ++rit;
            int rpos = rit != header_pos.cend() ?
                distance(header_pos.cbegin(), rit) + lpos : -1;
            itable.insert_col(cl, cd, rpos);
        }

        // update target_pos, does union of features_pos and
        // header_pos and put the result in new_header_pos
        vector<unsigned> new_header_pos;
        boost::set_union(header_pos, features_pos, back_inserter(new_header_pos));
        if (full_target_pos > 0) {
            if (full_target_pos < (int)new_header_pos.front()) // target is first
                target_pos = 0;
            else if (full_target_pos > (int)new_header_pos.back()) // target is last
                target_pos = -1;
            else {              // target is in between
                auto it = boost::adjacent_find(new_header_pos, [&](int l, int r) {
                        return l < full_target_pos && full_target_pos < r; });
                target_pos = distance(new_header_pos.begin(), ++it);
            }
        } else                  // target_pos is already at the 0 or
                                // -1 no need to change it
            OC_ASSERT(full_target_pos == target_pos, "smells a bug");
    }
}
        
// -------------------------------------------------------

void CTable::set_labels(const vector<string>& labels)
{
    olabel = labels.front();
    ilabels.clear();
    ilabels.insert(ilabels.begin(), labels.begin() + 1, labels.end());
}

vector<string> CTable::get_labels() const
{
    vector<string> labels = ilabels;
    labels.insert(labels.begin(), olabel);
    return labels;
}

unsigned CTable::uncompressed_size() const
{
    unsigned res = 0;
    for (const value_type& v : *this) {
        res += v.second.total_count();
    }
    return res;
}

type_node CTable::get_output_type() const
{
    return get_type_node(get_signature_output(tsig));
}

// -------------------------------------------------------

// XXX TODO replace this by the util p_norm function.
complete_truth_table::size_type
complete_truth_table::hamming_distance(const complete_truth_table& other) const
{
    OC_ASSERT(other.size() == size(),
              "complete_truth_tables size should be the same.");

    size_type res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += (*x++ != *y++);
    return res;
}

bool complete_truth_table::same_complete_truth_table(const combo_tree& tr) const
{
    const_iterator cit = begin();
    for (int i = 0; cit != end(); ++i, ++cit) {
        for (int j = 0; j < _arity; ++j)
            bmap[j] = bool_to_vertex((i >> j) % 2);
        if (*cit != vertex_to_bool(eval_binding(bmap, tr)))
            return false;
    }
    return true;
}
// -------------------------------------------------------

double OTEntropy(const OTable& ot)
{
    // Compute the probability distributions
    Counter<vertex, unsigned> counter(ot);
    vector<double> py(counter.size());
    double total = ot.size();
    transform(counter | map_values, py.begin(),
              [&](unsigned c) { return c/total; });
    // Compute the entropy
    return entropy(py);
}

}} // ~namespaces combo opencog
