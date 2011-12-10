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

#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/find.hpp>

#include <opencog/util/numeric.h>
#include <opencog/util/algorithm.h>

#include "ann.h"
#include "simple_nn.h"
#include "convert_ann_combo.h"

namespace opencog { namespace combo {

using namespace std;
using namespace boost;

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

truth_output_table::truth_output_table(const combo_tree& tr,
                                       const truth_input_table& tti,
                                       opencog::RandGen& rng) {
    for(bm_cit i = tti.begin(); i != tti.end(); ++i) {
        binding_map bmap = tti.get_binding_map(*i);
        vertex res = eval_throws_binding(rng, bmap, tr);
        OC_ASSERT(is_boolean(res), "res must be boolean");
        push_back(vertex_to_bool(res));
    }
}

truth_output_table::truth_output_table(const combo_tree& tr,
                                       const ctruth_table& ctt,
                                       opencog::RandGen& rng) {
    for(ctruth_table::const_iterator i = ctt.begin();
        i != ctt.end(); ++i) {
        binding_map bmap = ctt.get_binding_map(i->first);
        vertex res = eval_throws_binding(rng, bmap, tr);
        OC_ASSERT(is_boolean(res), "res must be boolean");
        push_back(vertex_to_bool(res));
    }
}

ctruth_table truth_table::compress() const {

    // Logger
    logger().debug("Compress the dataset, current size is %d", input.size());
    // ~Logger

    ctruth_table res(output.get_label(), input.get_labels());

    InputTable::const_iterator in_it = input.begin();
    OutputTable::const_iterator out_it = output.begin();
    for(; in_it != input.end(); ++in_it, ++out_it) {
        ctruth_table::iterator dup_in_it = res.find(*in_it);
        if(dup_in_it == res.end())
            res[*in_it] = make_pair(*out_it?0:1, *out_it?1:0);
        else {            // found, update duplicate's the output
            dup_in_it->second.first += *out_it?0:1;
            dup_in_it->second.second += *out_it?1:0;
        }
    }

    // Logger
    logger().debug("Size of the compressed dataset is %d", res.size());
    // ~Logger

    return res;
}

contin_input_table::contin_input_table(int sample_count, int arity,
                                       opencog::RandGen& rng, 
                                       double max_randvalue,
                                       double min_randvalue)
{
    //populate the matrix
    for (int i = 0; i < sample_count; ++i) {
        contin_vector cv;
        for (int j = 0; j < arity; ++j)
        cv.push_back((max_randvalue - min_randvalue) 
                     * rng.randdouble() + min_randvalue); 
        // input interval
        push_back(cv);
    }
}

contin_output_table::contin_output_table(const combo_tree& tr, const contin_input_table& cti,
                                         opencog::RandGen& rng)
{
    OC_ASSERT(!tr.empty());
    if(is_ann_type(*tr.begin())) { 
        // we treat ANN differently because they must be decoded
        // before being evaluated. Also note that if there are memory
        // neurones then the state of the network is evolving at each
        // input, so the order with contin_input_table does matter
        ann net = tree_transform().decodify_tree(tr);
        int depth = net.feedforward_depth();
        for(const_cm_it i = cti.begin(); i != cti.end(); ++i) {
            contin_vector tmp(*i);
            tmp.push_back(1.0); // net uses that in case the function
                                // to learn needs some kind of offset
            net.load_inputs(tmp);
            dorepeat(depth)
                net.propagate();
            push_back(net.outputs[0]->activation);
        }
    } else {
        for(const_cm_it i = cti.begin(); i != cti.end(); ++i) {
            binding_map bmap = cti.get_binding_map(*i);
            // assumption : all inputs and output of tr are contin_t
            // this assumption can be verified using infer_type_tree
            vertex res = eval_throws_binding(rng, bmap, tr);
            push_back(get_contin(res));
        }
    }
}

bool contin_output_table::operator==(const contin_output_table& ct) const
{
    if (get_label() != ct.get_label())
        return false;
    if (ct.size() == size()) {
        const_cv_it ct_i = ct.begin();
        for (const_cv_it i = begin(); i != end(); ++i, ++ct_i) {
            if (!isApproxEq(*i, *ct_i))
                return false;
        }
        return true;
    } else return false;
}

contin_t contin_output_table::abs_distance(const contin_output_table& other) const
{
    OC_ASSERT(other.size() == size(),
              "contin_output_tables should have the same size.");

    contin_t res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += fabs(*x++ -*y++);
    return res;
}

contin_t contin_output_table::sum_squared_error(const contin_output_table& other) const
{
    OC_ASSERT(other.size() == size(),
              "contin_output_tables should have the same size.");

    contin_t res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += sq(*x++ -*y++);
    return res;
}

contin_t contin_output_table::mean_squared_error(const contin_output_table& other) const
{
    OC_ASSERT(other.size() == size() && size() > 0,
              "contin_output_tables should have the same size > 0.");
    return sum_squared_error(other) / (contin_t)other.size();
}

contin_t contin_output_table::root_mean_square_error(const contin_output_table& other) const
{
    OC_ASSERT(other.size() == size() && size() > 0,
              "contin_output_tables should have the same size > 0.");
    return sqrt(mean_squared_error(other));
}

bool checkCarriageReturn(istream& in) {
    char next_c = in.get();
    if(next_c == '\r') // DOS format
        next_c = in.get();
    if(next_c == '\n')
        return true;
    return false;
}

void removeCarriageReturn(string& str) {
    size_t s = str.size();
    if((s > 0) && (str[s-1] == '\r'))
        str.resize(s-1);
}

void removeNonASCII(string& str) {
    while(str.size() && (unsigned char)str[0] > 127)
        str = str.substr(1);
}

arity_t istreamArity(istream& in) {
    string line;
    getline(in, line);    
    return tokenizeRowIO<string>(line).first.size();
}

ifstream* open_data_file(const string& fileName) {
    ifstream* in = new ifstream(fileName.c_str());
    if(!in->is_open()) {
        stringstream ss;
        ss << "Could not open " << fileName << endl;
        OC_ASSERT(false, ss.str());
    }
    return in;
}

vector<string> readInputLabels(const string& file, int pos) {
    auto_ptr<ifstream> in(open_data_file(file));
    string line;
    getline(*in, line);    
    return tokenizeRowIO<string>(line, pos).first;
}

arity_t dataFileArity(const string& fileName) {
    unique_ptr<ifstream> in(open_data_file(fileName));
    return istreamArity(*in);
}

/**
 * Check the token, if it is "0" or "1" then it is boolean, otherwise
 * it is contin. It is not 100% reliable of course and should be
 * improved.
 */
type_node infer_type_from_token(const string& token) {
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
    getline(*in, line);
    vector<string> labels = tokenizeRow<string>(line);
    unsigned int pos = distance(labels.begin(), boost::find(labels, target));
    if (pos < labels.size())
        return pos;
    else
        OC_ASSERT(false, "There is no such target feature %s in data file %s",
                  target.c_str(), fileName.c_str());
    return pos;
}

type_node inferDataType(const string& fileName)
{
    type_node res;
    unique_ptr<ifstream> in(open_data_file(fileName));
    string line;
    // check the last token of the first row
    getline(*in, line);
    res = infer_type_from_token(tokenizeRowIO<string>(line).second);
    if(res == id::ill_formed_type) { // check the last token of the second row
        getline(*in, line);
        res = infer_type_from_token(tokenizeRowIO<string>(line).second);
    }
    return res;
}

boost::tokenizer<boost::char_separator<char>>
get_row_tokenizer(std::string& line) {
    typedef boost::char_separator<char> seperator;
    typedef boost::tokenizer<seperator> tokenizer;
    typedef tokenizer::const_iterator tokenizer_cit;

    // remove weird symbols at the start of the line and carriage
    // return symbol (for DOS files)
    removeNonASCII(line);
    removeCarriageReturn(line);

    // tokenize line
    static const seperator sep(", \t");
    return tokenizer(line, sep);
}

}} // ~namespaces combo opencog
