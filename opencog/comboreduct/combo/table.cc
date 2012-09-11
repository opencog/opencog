/** table.cc ---
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
#include "table.h"

#include <iomanip>

#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/transform.hpp>

#include <opencog/util/dorepeat.h>
#include <opencog/util/Logger.h>

#include "ann.h"
#include "simple_nn.h"
#include "convert_ann_combo.h"

namespace opencog { namespace combo {

using namespace std;
using namespace boost;
using namespace boost::adaptors;

ITable::ITable() {}

ITable::ITable(const ITable::super& mat, vector<string> il)
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

void ITable::set_labels(const vector<string>& il)
{
    labels = il;
}

const vector<string>& ITable::get_labels() const
{
    if (labels.empty() and !super::empty()) // return default labels
        labels = get_default_labels();
    return labels;
}

// -------------------------------------------------------

OTable::OTable(const string& ol)
    : label(ol) {}

OTable::OTable(const super& ot, const string& ol)
    : super(ot), label(ol) {}

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
        foreach(const vertex_seq& vv, itable) {
            vector<contin_t> tmp(vv.size());
            transform(vv, tmp.begin(), get_contin);
            tmp.push_back(1.0); // net uses that in case the function
                                // to learn needs some kind of offset
            net.load_inputs(tmp);
            dorepeat(depth)
                net.propagate();
            push_back(net.outputs[0]->activation);
        }
    } else {
        foreach(const vertex_seq& vs, itable)
            push_back(eval_throws_binding(vs, tr));
    }
}

OTable::OTable(const combo_tree& tr, const CTable& ctable, const string& ol)
    : label(ol)
{
    arity_set as = get_argument_abs_idx_set(tr);
    for_each(ctable | map_keys, [&](const vertex_seq& vs) {
            this->push_back(eval_throws_binding(vs, tr));
        });
}

void OTable::set_label(const string& ol)
{
    label = ol;
}

const string& OTable::get_label() const
{
    return label;
}

// -------------------------------------------------------

bool OTable::operator==(const OTable& rhs) const
{
    const static contin_t epsilon = 1e-12;
    for(auto lit = begin(), rit = rhs.begin(); lit != end(); ++lit, ++rit) {
        if(is_contin(*lit) && is_contin(*rit)) {
            if(!isApproxEq(get_contin(*lit), get_contin(*rit), epsilon))
                return false;
        }
        else if(*lit != *rit)
            return false;
    }
    return rhs.get_label() == label;
}

contin_t OTable::abs_distance(const OTable& ot) const
{
    OC_ASSERT(ot.size() == size());
    contin_t res = 0;
    for(const_iterator x = begin(), y = ot.begin(); x != end();)
        res += fabs(get_contin(*(x++)) - get_contin(*(y++)));
    return res;
}

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

Table::Table() {}

Table::Table(const OTable& otable_, const ITable& itable_, const type_tree& tt_)
    : tt(tt_), itable(itable_), otable(otable_) {}

Table::Table(const combo_tree& tr, int nsamples,
             contin_t min_contin, contin_t max_contin) :
    tt(infer_type_tree(tr)), itable(tt, nsamples, min_contin, max_contin),
    otable(tr, itable) {}

vector<string> Table::get_labels() const
{
    vector<string> labels = itable.get_labels();
    labels.insert(labels.begin(), otable.get_label());
    return labels;
}

CTable Table::compressed() const
{
    // Logger
    logger().debug("Compress the dataset, current size is %d", itable.size());
    // ~Logger

    CTable res(otable.get_label(), itable.get_labels());
    // assign type_tree
    res.tt = tt;

    ITable::const_iterator in_it = itable.begin();
    OTable::const_iterator out_it = otable.begin();
    for(; in_it != itable.end(); ++in_it, ++out_it)
        ++res[*in_it][*out_it];

    // Logger
    logger().debug("Size of the compressed dataset is %d", res.size());
    // ~Logger

    return res;
}

// -------------------------------------------------------

vector<string> CTable::get_labels() const
{
    vector<string> labels = ilabels;
    labels.insert(labels.begin(), olabel);
    return labels;
}

// -------------------------------------------------------

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
