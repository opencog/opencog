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

#include <ctype.h>
#include <math.h>
#include <stdlib.h>

#include <iomanip>

#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/adjacent_find.hpp>
#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <opencog/util/dorepeat.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lazy_random_selector.h>

#include "../combo/ann.h"
#include "../combo/simple_nn.h"
#include "../combo/convert_ann_combo.h"

#include "table.h"
#include "table_io.h"

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

/// Construct an ITable holding a single column, the column from the OTable.
ITable::ITable(const OTable& ot)
{
    insert_col(ot.get_label(), ot);

    type_seq typs;
    typs.push_back(ot.get_type());
    set_types(typs);
}

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
    throw(IndexErrorException)
{
    int off = get_column_offset(name);
    if (-1 == off)
        throw IndexErrorException(TRACE_INFO,
            "Can't delete, unknown column name: %s", name.c_str());

    // Delete the column
    for (multi_type_seq& row : *this)
        row.erase_at(off);

    // Delete the label as well.
    string rv;
    if (not labels.empty()) {
        rv = *(labels.begin() + off);
        labels.erase(labels.begin() + off);
    }

    if (not types.empty())
        types.erase(types.begin() + off);

    return rv;
}


void ITable::delete_columns(const vector<string>& ignore_features)
    throw(IndexErrorException)
{
    for (const string& feat : ignore_features)
        delete_column(feat);
}

////////////
// OTable //
////////////

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
    OC_ASSERT(not tr.empty());
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
    if (id::contin_type == type and id::contin_type == ot.type) {
        for (const_iterator x = begin(), y = ot.begin(); x != end();) 
            res += fabs(get_contin(*(x++)) - get_contin(*(y++)));
    }
    else
    if (id::boolean_type == type and id::boolean_type == ot.type) {
        for (const_iterator x = begin(), y = ot.begin(); x != end();) 
            res += (contin_t) (get_builtin(*(x++)) != get_builtin(*(y++)));
    }
    else
    if (id::enum_type == type and id::enum_type == ot.type) {
        for (const_iterator x = begin(), y = ot.begin(); x != end();) 
            res += (contin_t) (get_enum_type(*(x++)) != get_enum_type(*(y++)));
    }
    else
        throw InconsistenceException(TRACE_INFO,
            "Can't compare, mismatched column types.");
    return res;
}

// XXX TODO replace this by the util p_norm function.
contin_t OTable::sum_squared_error(const OTable& ot) const
{
    OC_ASSERT(ot.size() == size());
    contin_t res = 0;
    for (const_iterator x = begin(), y = ot.begin(); x != end();)
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

////////////
// TTable //
////////////

TTable::TTable(const string& tl)
    : label(tl) {}

TTable::TTable(const super& tt, const string& tl)
    : super(tt), label(tl) {}

void TTable::set_label(const string& tl)
{
    label = tl;
}

const string& TTable::get_label() const
{
    return label;
}

TTable::value_type TTable::from_string(const std::string& timestamp_str) {
    return boost::gregorian::from_string(timestamp_str);
}

std::string TTable::to_string(const TTable::value_type& timestamp) {
    return boost::gregorian::to_iso_extended_string(timestamp);
}

///////////
// Table //
///////////

Table::Table() : target_pos(0), timestamp_pos(0) {}

Table::Table(const OTable& otable_, const ITable& itable_)
    : itable(itable_), otable(otable_), target_pos(0), timestamp_pos(0) {}

Table::Table(const combo_tree& tr, int nsamples,
             contin_t min_contin, contin_t max_contin) :
    itable(infer_type_tree(tr), nsamples, min_contin, max_contin),
    otable(tr, itable), target_pos(0), timestamp_pos(0) {}

vector<string> Table::get_labels() const
{
    vector<string> labels = itable.get_labels();
    labels.insert(labels.begin(), otable.get_label());
    return labels;
}

// -------------------------------------------------------

CTable Table::compressed(const std::string weight_col) const
{
    logger().debug("Compress the dataset, current size is %d", itable.size());

    // If no weight column, then its straight-forward
    if (weight_col.empty()) {
        CTable res(otable.get_label(), itable.get_labels(), get_signature());

        ITable::const_iterator in_it = itable.begin();
        OTable::const_iterator out_it = otable.begin();
        if (ttable.empty())
            for(; in_it != itable.end(); ++in_it, ++out_it)
                ++res[*in_it][TimedValue(*out_it)];
        else {
            TTable::const_iterator time_it = ttable.begin();
            for(; in_it != itable.end(); ++in_it, ++out_it, ++time_it)
                ++res[*in_it][TimedValue(*out_it, *time_it)];
        }
        logger().debug("Size of the compressed dataset is %u", res.size());
        return res;
    }
    else {
        // Else, remove the weight column from the input;
        // we don't want to use it as an independent feature.
        ITable trimmed(itable);
        trimmed.delete_column(weight_col);

        CTable res(otable.get_label(), trimmed.get_labels(), get_signature());

        size_t widx = itable.get_column_offset(weight_col);
        ITable::const_iterator w_it = itable.begin();
        ITable::const_iterator in_it = trimmed.begin();
        OTable::const_iterator out_it = otable.begin();
        if (ttable.empty()) {
            for (; in_it != trimmed.end(); ++in_it, ++out_it, ++w_it)
            {
                vertex v = w_it->get_at<vertex>(widx);
                contin_t weight = get_contin(v);
                res[*in_it][TimedValue(*out_it)] += weight;
            }
        }
        else {
            TTable::const_iterator time_it = ttable.begin();
            for (; in_it != trimmed.end(); ++in_it, ++out_it, ++w_it, ++time_it)
            {
                vertex v = w_it->get_at<vertex>(widx);
                contin_t weight = get_contin(v);
                res[*in_it][TimedValue(*out_it, *time_it)] += weight;
            }
        }
        logger().debug("Size of the compressed dataset is %d", res.size());
        return res;
    }
}

// -------------------------------------------------------

/**
 * Get indices (aka positions or offsets) of a list of labels given a
 * header. The labels can be sequenced in any order, it will always
 * return the order consistent with the header.
 */
vector<unsigned> get_indices(const vector<string>& labels,
                             const vector<string>& header)
{
    vector<unsigned> res;
    for (unsigned i = 0; i < header.size(); ++i)
        if (std::find(labels.begin(), labels.end(), header[i]) != labels.end())
            res.push_back(i);
    return res;
}

std::vector<contin_t> discretize_contin_feature(contin_t min,
                                                contin_t max)
{
    std::vector<contin_t> res;
    contin_t interval = (max - min)/TARGET_DISCRETIZED_BINS_NUM;
    for (unsigned i = 0; i < TARGET_DISCRETIZED_BINS_NUM; ++i)
        res.push_back(min+i*interval);
    return res;
}

builtin get_discrete_bin(std::vector<contin_t> disc_intvs, contin_t val)
{
    unsigned i;
    for (i = 1; i < TARGET_DISCRETIZED_BINS_NUM; i++)
    {
        if (val < disc_intvs[i])
            break;
    }
    return (builtin)i;
}

unsigned get_index(const string& label, const vector<string>& header)
{
    return std::distance(header.begin(), std::find(header.begin(), header.end(), label));
}

bool Table::operator==(const Table& rhs) const {
    return itable == rhs.itable and otable == rhs.otable and ttable == rhs.ttable
        and target_pos == rhs.target_pos and timestamp_pos == rhs.timestamp_pos;
}
        
/////////////////
// TimeCounter //
/////////////////

count_t TimedCounter::get(const vertex& v) const {
    count_t res = 0;
    for (const auto& vtc : *this)
        if (vtc.first.value == v)
            res += vtc.second;
    return res;
}

Counter<vertex, count_t> TimedCounter::untimedCounter() const {
    Counter<vertex, count_t> vc;
    for (const auto& vtc : *this)
        vc[vtc.first.value] += vtc.second;
    return vc;
}

vertex TimedCounter::most_frequent() const {
    return untimedCounter().most_frequent();
}

////////////
// CTable //
////////////

CTable::CTable(const std::string& _olabel)
    : olabel(_olabel) {}

CTable::CTable(const string_seq& labs, const type_tree& tt)
    : tsig(tt), olabel(labs[0]), ilabels(labs)
{
    ilabels.erase(ilabels.begin());
}

CTable::CTable(const std::string& _olabel, const string_seq& _ilabels,
               const type_tree& tt)
    : tsig(tt), olabel(_olabel), ilabels(_ilabels)
{}


void CTable::remove_rows(const set<unsigned>& idxs)
{
    // iterator of the set of row indexes to remove
    auto idx_it = idxs.begin();

    // iterator index of the CTable from the perspective of an
    // uncompressed table
    unsigned i = 0;

    // For each row check if some indexes are within uncompressed
    // begining and end of that row and decrease their counts if
    // so
    for (auto row_it = begin();
         row_it != end() and idx_it != idxs.end();) {
        auto& outputs = row_it->second;
// XXX this cannot possibly be correct, the total count is in general
// a fraction, not an integer; it is merely the sum of the weights
// of the rows. It is NOT equal to the toal number of rows!
// I cannot figure out what this algo is trying to do, so I can't
// actually fix it :-(
        count_t total_row_weights = outputs.total_count();
        count_t truncate = floor(total_row_weights);
OC_ASSERT(0.0 == (total_row_weights - truncate), "This algo is broken!");
        unsigned i_end = i + ((unsigned) truncate);
        if (i <= *idx_it and *idx_it < i_end) {
            for (auto v_it = outputs.begin();
                 v_it != outputs.end() and idx_it != idxs.end();) {
                OC_ASSERT(i <= *idx_it, "There must be a bug");
                // Remove all overlapping indexes with v and
                // advance idx_it of that amount
                unsigned i_v_end = i + v_it->second;
                while (idx_it != idxs.end() and *idx_it < i_v_end) {
                    OC_ASSERT(v_it->second > 0, "There must be a bug");
                    --v_it->second;
                    ++idx_it;
                }

                // Increment i with the count of that value
                i = i_v_end;

                // Check if the count went to zero, if so remove
                // v_it entirely
                if (v_it->second == 0)
                    v_it = outputs.erase(v_it);
                else
                    ++v_it;
            }

            // Check if the output is empty, and if so remove the
            // row entirely
            if (row_it->second.empty())
                row_it = erase(row_it);
            else
                ++row_it;
        } else {
            ++row_it;
        }
        i = i_end;
    }
}

void CTable::remove_rows_at_times(const set<TTable::value_type>& timestamps)
{
    for (const TTable::value_type& timestamp : timestamps)
        remove_rows_at_time(timestamp);
}

void CTable::remove_rows_at_time(const TTable::value_type& timestamp)
{
    for (auto row_it = begin(); row_it != end();) {
        auto& outputs = row_it->second;

        // Remove all output values at timestamp
        for (auto v_it = outputs.begin(); v_it != outputs.end();) {
            if (v_it->first.timestamp == timestamp)
                v_it = outputs.erase(v_it);
            else
                ++v_it;
        }

        // Check if the output is empty, and if so remove the
        // row entirely
        if (row_it->second.empty())
            row_it = erase(row_it);
        else
            ++row_it;
    }
}

set<TTable::value_type> CTable::get_timestamps() const
{
    set<TTable::value_type> res;
    for (const CTable::value_type& row : *this)
        for (const auto& vtc : row.second)
            if (vtc.first.timestamp != boost::gregorian::date())
                res.insert(vtc.first.timestamp);

    return res;
}

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

const string& CTable::get_output_label() const
{
    return olabel;
}

const string_seq& CTable::get_input_labels() const
{
    return ilabels;
}

void CTable::set_signature(const type_tree& tt)
{
    tsig = tt;
}

const type_tree& CTable::get_signature() const
{
    return tsig;
}

count_t CTable::uncompressed_size() const
{
    count_t res = 0.0;
    for (const value_type& v : *this) {
        res += v.second.total_count();
    }
    return res;
}

type_node CTable::get_output_type() const
{
    return get_type_node(get_signature_output(tsig));
}

CTableTime CTable::ordered_by_time() const
{
    // Turn the input to timestamped output map into timetamp to
    // output map
    CTableTime res;
    for (const auto& v : *this)
        for (const auto& tcv : v.second)
            res[tcv.first.timestamp] +=
                Counter<vertex, count_t>({{tcv.first.value, tcv.second}});
    return res;
}

void CTable::balance()
{
    type_node otype = get_output_type();
    if (otype == id::boolean_type or otype == id::enum_type) {
        // Get total count for each class (called Ni in comment below)
        Counter<vertex, count_t> class_count;
        for (auto iorow : *this)
            class_count += iorow.second.untimedCounter();

        count_t usize = uncompressed_size(), n = class_count.size();
        // N1 + ... + Nn = usize
        //
        // where Ni = ci1 + ... cim, with cij the count of each row j
        // of class i.
        //
        // We want Ni' = usize / n, the new count of class i.
        //
        // where Ni' = ci1' + ... + cim', with cij' the count of each
        // row of class i.
        //
        // cij' = ci * cij
        //
        // ci * ci1 + ... + ci * cim = usize / n
        // ci = (usize/n) / Ni
        for (auto iorow : *this)
            for (auto tvc : iorow.second)
                tvc.second *= (usize / n) / class_count[tvc.first.value];
    } else {
        logger().warn() << "CTable::balance() - "
                        << "cannot balance non discrete output type "
                        << otype;
    }
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

/////////////////////
// Subsample table //
/////////////////////

// Remove enough rows randomly so that the table has only nrows
void subsampleTable(unsigned nrows, ITable& it, OTable& ot, TTable& tt)
{
    OC_ASSERT(it.empty() || ot.empty() || it.size() == ot.size());
    OC_ASSERT(ot.empty() || tt.empty() || ot.size() == tt.size());
    OC_ASSERT(tt.empty() || it.empty() || tt.size() == it.size());
    unsigned size = std::max(it.size(), std::max(ot.size(), tt.size()));
    if(nrows < size) {
        unsigned nremove = size - nrows;
        dorepeat(nremove) {
            unsigned int ridx = randGen().randint(size);
            if (!it.empty())
                it.erase(it.begin()+ridx);
            if (!ot.empty())
                ot.erase(ot.begin()+ridx);
            if (!tt.empty())
                tt.erase(tt.begin()+ridx);
        }
    }
}

void subsampleTable(float ratio, Table& table)
{
    OC_ASSERT(0.0 <= ratio and ratio <= 1.0,
              "Ratio must be in [0.0, 1.0], but is %f", ratio);
    subsampleTable(ratio * table.size(), table.itable, table.otable, table.ttable);
}

void subsampleCTable(float ratio, CTable& ctable)
{
    OC_ASSERT(0.0 <= ratio and ratio <= 1.0,
              "Ratio must be in [0.0, 1.0], but is %f", ratio);
    std::set<unsigned> rm_row_idxs;
    unsigned ctable_usize = ctable.uncompressed_size(),
        nremove = (1.0 - ratio) * ctable_usize;
    lazy_random_selector rm_selector(ctable_usize);
    dorepeat(nremove)
        rm_row_idxs.insert(rm_selector.select());
    ctable.remove_rows(rm_row_idxs);
    subsampleCTable(ratio * ctable.uncompressed_size(), ctable);
}

////////////////////////
// Mutual Information //
////////////////////////

double OTEntropy(const OTable& ot)
{
    // Compute the probability distributions
    Counter<vertex, count_t> counter(ot);
    vector<double> py(counter.size());
    double total = ot.size();
    transform(counter | map_values, py.begin(),
              [&](count_t c) { return c/total; });
    // Compute the entropy
    return entropy(py);
}

}} // ~namespaces combo opencog
