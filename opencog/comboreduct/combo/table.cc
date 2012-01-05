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

#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/adaptor/map.hpp>

#include <opencog/util/numeric.h>
#include <opencog/util/algorithm.h>

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

ITable::ITable(const type_tree& tt, RandGen& rng, int nsamples,
               contin_t min_contin, contin_t max_contin) {
    arity_t barity = boolean_arity(tt), carity = contin_arity(tt);

    if(nsamples < 0)
        nsamples = std::max(pow2(barity), sample_count(carity));

    // in that case the boolean inputs are not picked randomly but
    // instead are enumerated
    bool comp_tt = nsamples == (int)pow2(barity);

    //populate the matrix
    auto root = tt.begin();
    for(int i = 0; i < nsamples; ++i) {
        size_t bidx = 0;
        vertex_seq vv;
        foreach(type_node n, make_pair(root.begin(), root.last_child()))
            if(n == id::boolean_type)
                if(comp_tt)
                    vv.push_back(bool_to_vertex(i & (1 << bidx++)));
                else
                    vv.push_back(bool_to_vertex(rng.randint(2)));
            else if(n == id::contin_type)
                vv.push_back((max_contin - min_contin)
                             * rng.randdouble() + min_contin);

        // input interval
        push_back(vv);
    }
}

void ITable::set_labels(vector<string> il) {
    labels = il;
}

const vector<string>& ITable::get_labels() const {
    if(labels.empty() and !super::empty()) { // return default labels
        static const vector<string> dl(get_default_labels());
        return dl;
    } else return labels;
}

OTable::OTable(const string& ol) : label(ol) {}

OTable::OTable(const super& ot, const string& ol) : super(ot), label(ol) {}

OTable::OTable(const combo_tree& tr, const ITable& itable, RandGen& rng,
               const string& ol) : label(ol) {
    OC_ASSERT(!tr.empty());
    if(is_ann_type(*tr.begin())) {
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
        foreach(const vertex_seq& vv, itable) {
            binding_map bmap = itable.get_binding_map(vv);
            push_back(eval_throws_binding(rng, bmap, tr));
        }
    }
}

OTable::OTable(const combo_tree& tr, const CTable& ctable, RandGen& rng,
               const string& ol) : label(ol) {
    for_each(ctable | map_keys, [&](const vertex_seq& vs) {
            binding_map bmap = ctable.get_binding_map(vs);
            this->push_back(eval_throws_binding(rng, bmap, tr)); });
}

void OTable::set_label(const string& ol) {
    label = ol;
}

const string& OTable::get_label() const {
    return label;
}

Table::Table() {}

Table::Table(const combo_tree& tr, RandGen& rng, int nsamples,
             contin_t min_contin, contin_t max_contin) :
    tt(infer_type_tree(tr)), itable(tt, rng, nsamples, min_contin, max_contin),
    otable(tr, itable, rng) {}

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

bool complete_truth_table::same_complete_truth_table(const combo_tree& tr) const {
    const_iterator cit = begin();
    for (int i = 0; cit != end(); ++i, ++cit) {
        for (int j = 0; j < _arity; ++j)
            bmap[j + 1] = bool_to_vertex((i >> j) % 2);
        if(*cit != vertex_to_bool(eval_binding(*_rng, bmap, tr)))
            return false;
    }
    return true;
}

CTable Table::compress() const {

    // Logger
    logger().debug("Compress the dataset, current size is %d", itable.size());
    // ~Logger

    CTable res(otable.get_label(), itable.get_labels());

    ITable::const_iterator in_it = itable.begin();
    OTable::const_iterator out_it = otable.begin();
    for(; in_it != itable.end(); ++in_it, ++out_it)
        ++res[*in_it][*out_it];

    // Logger
    logger().debug("Size of the compressed dataset is %d", res.size());
    // ~Logger

    return res;
}

bool OTable::operator==(const OTable& rhs) const {
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

bool checkCarriageReturn(istream& in)
{
    char next_c = in.get();
    if (next_c == '\r') // DOS format
        next_c = in.get();
    if (next_c == '\n')
        return true;
    return false;
}

void removeCarriageReturn(string& str)
{
    size_t s = str.size();
    if ((s > 0) && (str[s-1] == '\r'))
        str.resize(s-1);
}

void removeNonASCII(string& str)
{
    while(str.size() && (unsigned char)str[0] > 127)
        str = str.substr(1);
}

// Return true if the character is one of the standard comment
// delimiters.  Here, we define a 'standard delimiter' as one
// of hash, bang or semicolon.
bool is_comment(const char c)
{
    if ('#' == c) return true;
    if (';' == c) return true;
    if ('!' == c) return true;
    if ('\n' == c) return true;
    if ('\r' == c) return true;
    if (0 == c) return true;
    return false;
}

//* Get one line of actual data.
// This ignores lines that start with a 'standard comment char'
//
// TODO: This routine should be extended so that comments elsewhere in
// the file are also ignored...
//
// The signature of this routine is the same as std:getline()
//
istream &get_data_line(istream& is, string& line)
{
    while (1)
    {
        getline(is, line);
        if (!is) return is;
        if (is_comment(line[0])) continue;
        return is;
    }
}

arity_t istreamArity(istream& in)
{
    string line;
    get_data_line(in, line);
    return tokenizeRowIO<string>(line).first.size();
}

ifstream* open_data_file(const string& fileName)
{
    ifstream* in = new ifstream(fileName.c_str());
    if(!in->is_open()) {
        stringstream ss;
        ss << "ERROR: Could not open " << fileName << endl;
        OC_ASSERT(false, ss.str());
    }
    return in;
}

vector<string> readInputLabels(const string& file, int pos)
{
    auto_ptr<ifstream> in(open_data_file(file));
    string line;
    get_data_line(*in, line);
    return tokenizeRowIO<string>(line, pos).first;
}

arity_t dataFileArity(const string& fileName)
{
    unique_ptr<ifstream> in(open_data_file(fileName));
    return istreamArity(*in);
}

//* Return true if the table seems to have a header line in it.
bool has_header(const string& dataFileName)
{
    unique_ptr<ifstream> in(open_data_file(dataFileName));
    string line;
    get_data_line(*in, line);
    type_node n = infer_type_from_token(tokenizeRow<string>(line).front());
    return n == id::ill_formed_type;
}

type_node infer_type_from_token(const string& token)
{
    if(token == "0" || token == "1")
        return id::boolean_type;
    else {
        try {
            lexical_cast<contin_t>(token);
            return id::contin_type;
        }
        catch(...) {
            return id::ill_formed_type;
        }
    }
}

int findTargetFeaturePosition(const string& fileName, const string& target)
{
    unique_ptr<ifstream> in(open_data_file(fileName));
    string line;
    get_data_line(*in, line);
    vector<string> labels = tokenizeRow<string>(line);
    unsigned pos = distance(labels.begin(), find(labels, target));
    OC_ASSERT(pos < labels.size(),
              "ERROR: There is no column labelled \"%s\" in data file %s",
              target.c_str(), fileName.c_str());
    return pos;
}

type_tree infer_row_type_tree(const pair<vector<string>, string>& row)
{
    type_tree tt(id::lambda_type);
    auto root = tt.begin();

    foreach(const string& s, row.first) {
        type_node n = infer_type_from_token(s);
        if(n == id::ill_formed_type)
            return type_tree(id::ill_formed_type);
        else
            tt.append_child(root, n);
    }
    type_node n = infer_type_from_token(row.second);
    if(n == id::ill_formed_type)
        return type_tree(id::ill_formed_type);
    else
        tt.append_child(root, n);

    return tt;
}

type_tree infer_data_type_tree(const string& fileName, int pos)
{
    unique_ptr<ifstream> in(open_data_file(fileName));
    string line;
    get_data_line(*in, line);
    if (has_header(fileName))
        get_data_line(*in, line);
    type_tree res = infer_row_type_tree(tokenizeRowIO<string>(line));
    OC_ASSERT(is_well_formed(res));
    return res;
}

boost::tokenizer<boost::char_separator<char>>
get_row_tokenizer(string& line)
{
    typedef boost::char_separator<char> seperator;
    typedef boost::tokenizer<seperator> tokenizer;
    typedef tokenizer::const_iterator tokenizer_cit;

    // remove weird symbols at the start of the line and carriage
    // return symbol (for DOS files)
    removeNonASCII(line);
    removeCarriageReturn(line);

    // tokenize line; current allow tabs, commas, blanks.
    static const seperator sep(",\t ");
    return tokenizer(line, sep);
}

vertex token_to_vertex(const string& token)
{
    if(token == "0")
        return id::logical_false;
    else if(token == "1")
        return id::logical_true;
    else
        return lexical_cast<contin_t>(token);
}

istream& istreamTable(istream& in, ITable& it, OTable& ot,
                      bool has_header, const type_tree& tt, int pos)
{
    string line;
    arity_t arity = type_tree_arity(tt);

    if (has_header) {
        get_data_line(in, line);
        pair<vector<string>, string> ioh = tokenizeRowIO<string>(line, pos);
        it.set_labels(ioh.first);
        ot.set_label(ioh.second);
        OC_ASSERT(arity == (arity_t)ioh.first.size(),
                  "ERROR: Input file header/data declaration mismatch: "
                  "The header has %u columns while the first row has "
                  "%d columns.\n",
                  ioh.first.size(), arity);
    }

    while (get_data_line(in, line)) {

        // tokenize the line and fill the input vector and output
        pair<vector<string>, string> io = tokenizeRowIO<string>(line, pos);

        // check arity
        OC_ASSERT(arity == (arity_t)io.first.size(),
                  "ERROR: Input file inconsistent: the row %u has %u "
                  "columns while the first row has %d columns.  All "
                  "rows should have the same number of columns.\n",
                  ot.size(), io.first.size(), arity);

        // fill table
        vertex_seq ivs(arity);
        transform(io.first, ivs.begin(), token_to_vertex);
        it.push_back(ivs);
        ot.push_back(token_to_vertex(io.second));
    }
    return in;
}

void istreamTable(const string& file_name, ITable& it, OTable& ot, int pos)
{
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());
    istreamTable(in, it, ot, has_header(file_name),
                 infer_data_type_tree(file_name), pos);
}

Table istreamTable(const string& file_name, int pos)
{
    Table res;
    istreamTable(file_name, res.itable, res.otable, pos);
    return res;
}

ostream& ostreamTableHeader(ostream& out, const ITable& it, const OTable& ot)
{
    out << ot.get_label() << ",";
    return ostreamlnContainer(out, it.get_labels(), ",");
}

string vertex_to_str(const vertex& v)
{
    stringstream ss;
    if(is_boolean(v))
        ss << vertex_to_bool(v);
    else
        ss << v;
    return ss.str();
}

ostream& ostreamTable(ostream& out, const ITable& it, const OTable& ot)
{
    // print header
    ostreamTableHeader(out, it, ot);
    // print data
    OC_ASSERT(it.size() == ot.size());
    for(size_t row = 0; row < it.size(); ++row) {
        // print output
        out << vertex_to_str(ot[row]) << ",";
        // print inputs
        const auto& irow = it[row];
        for(unsigned i = 0; i < irow.size();) {
            out << vertex_to_str(irow[i]);
            ++i;
            if(i < irow.size())
                out << ",";
        }
        out << endl;
    }
    return out;
}

ostream& ostreamTable(ostream& out, const Table& table)
{
    return ostreamTable(out, table.itable, table.otable);
}

void ostreamTable(const string& file_name, const ITable& it, const OTable& ot) {
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ofstream out(file_name.c_str());
    OC_ASSERT(out.is_open(), "Could not open %s", file_name.c_str());
    ostreamTable(out, it, ot);
}
void ostreamTable(const string& file_name, const Table& table) {
    ostreamTable(file_name, table.itable, table.otable);
}
ostream& ostreamCTableHeader(ostream& out, const CTable& ct) {
    out << ct.olabel << ",";
    return ostreamlnContainer(out, ct.ilabels, ",");
}
ostream& ostreamCTable(ostream& out, const CTable& ct) {
    // print header
    ostreamCTableHeader(out, ct);
    // print data
    foreach(const auto& v, ct) {
        // print map of outputs
        out << "{";
        for(auto it = v.second.begin(); it != v.second.end();) {
            out << it->first << ":" << it->second;
            if(++it != v.second.end())
                out << ",";
        }
        out << "},";
        // print inputs
        ostreamlnContainer(out, v.first, ",");
    }
    return out;
}

void subsampleTable(ITable& it, OTable& ot, unsigned nsamples, RandGen& rng)
{
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

void subsampleTable(Table& table, unsigned nsamples, RandGen& rng)
{
    subsampleTable(table.itable, table.otable, nsamples, rng);
}

void subsampleTable(ITable& it, unsigned nsamples, RandGen& rng) {
    if(nsamples < it.size()) {
        unsigned int nremove = it.size() - nsamples;
        dorepeat(nremove) {
            unsigned int ridx = rng.randint(it.size());
            it.erase(it.begin()+ridx);
        }
    }
}

ostream& operator<<(ostream& out, const ITable& it)
{
    ostreamlnContainer(out, it.get_labels(), ",");
    foreach(const vertex_seq& row, it) {
        for(unsigned i = 0; i < row.size();) {
            out << vertex_to_str(row[i]);
            ++i;
            if(i < row.size())
                out << ",";
        }
        out << endl;
    }
    return out;
}

ostream& operator<<(ostream& out, const OTable& ot)
{
    if(!ot.get_label().empty())
        out << ot.get_label() << endl;
    foreach(const vertex& v, ot)
        out << vertex_to_str(v) << endl;
    return out;
}

ostream& operator<<(ostream& out, const complete_truth_table& tt)
{
    return ostreamContainer(out, tt);
}

}} // ~namespaces combo opencog
