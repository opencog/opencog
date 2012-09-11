/** table_io.cc ---
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
#include <iomanip>
#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/irange.hpp>
#include <boost/tokenizer.hpp>

#include <opencog/util/dorepeat.h>
#include <opencog/util/oc_omp.h>

#include "table.h"
#include "table_io.h"

namespace opencog { namespace combo {

using namespace std;
using namespace boost;

// -------------------------------------------------------

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

//* Remove non-ascii characters at the bigining of the line, only.
void removeNonASCII(string& str)
{
    while (str.size() && (unsigned char)str[0] > 127)
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
// TODO: This routine should be extended so that comments that start
// somewhere other than column 0 are also ignored.
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

vector<string> readInputLabels(const string& file, int pos,
                               const vector<int>& ignore_features)
{
    auto_ptr<ifstream> in(open_data_file(file));
    string line;
    get_data_line(*in, line);
    return tokenizeRowIO<string>(line, pos, ignore_features).first;
}

arity_t dataFileArity(const string& fileName)
{
    unique_ptr<ifstream> in(open_data_file(fileName));
    return istreamArity(*in);
}

vector<string> loadHeader(const string& file_name)
{
    unique_ptr<ifstream> in(open_data_file(file_name));
    string line;
    get_data_line(*in, line);
    return tokenizeRow<string>(line);
}

//* Return true if the table seems to have a header line in it.
bool hasHeader(const string& dataFileName)
{
    type_node n = infer_type_from_token(loadHeader(dataFileName).front());

    // Well, first row might be enums, or headers...
    return (n == id::ill_formed_type) || (n == id::enum_type);
}

/**
 * Given an input string, guess the type of the string.
 * Inferable types are: boolean, contin and enum.
 */
type_node infer_type_from_token(const string& token)
{
    /* Prefered representation is T's and 0's, to maximize clarity,
     * readability.  Numeric values are easily confused with contin
     * type.
     */ 
    if (token == "0" || 
        token == "1" ||
        token == "T" ||
        token == "F" ||
        token == "t" ||
        token == "f")
        return id::boolean_type;

    // If it starts with an alphabetic character, assume its a string
    else if (isalpha(token[0]))
        return id::enum_type;

    // Hope that we can cast this to a float point number.
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

/**
 * Find the column numbers associated with the names features
 * 
 * If the target begins with an alpha character, it is assumed to be a
 * column label. We return the column number; 0 is the left-most column.
 *
 * If the target is numeric, just assume that it is a column number.
 */
vector<int> find_features_positions(const string& fileName,
                                    const vector<string>& features)
{
    unique_ptr<ifstream> in(open_data_file(fileName));
    string line;
    get_data_line(*in, line);
    vector<string> labels = tokenizeRow<string>(line);
    vector<int> positions;
    
    foreach (const string& f, features) {
        // If its just numeric, go with it; double-check the value.
        int pos;
        if (isdigit(f[0])) {
            pos = atoi(f.c_str());
            pos --;  // let users number columns starting at 1.
            OC_ASSERT((pos < (int)labels.size()) || (0 <= pos),
                      "ERROR: The column number \"%s\" doesn't exist in data file %s",
                      f.c_str(), fileName.c_str());
        }
        else {
            // Not numeric; search for the column name
            pos = distance(labels.begin(), find(labels, f));
            OC_ASSERT((pos < (int)labels.size()) || (0 <= pos),
                      "ERROR: There is no column labelled \"%s\" in data file %s",
                      f.c_str(), fileName.c_str());
        }
        positions.push_back(pos);
    }
    return positions;
}

// Like above but takes only a single feature
int find_feature_position(const string& fileName, const string& feature)
{
    vector<string> features{feature};
    return find_features_positions(fileName, features).back();
}
        
type_tree infer_row_type_tree(const pair<vector<string>, string>& row)
{
    type_tree tt(id::lambda_type);
    auto root = tt.begin();

    foreach (const string& s, row.first) {
        type_node n = infer_type_from_token(s);
        if (n == id::ill_formed_type)
            return type_tree(id::ill_formed_type);
        else
            tt.append_child(root, n);
    }
    type_node n = infer_type_from_token(row.second);
    if (n == id::ill_formed_type)
        return type_tree(id::ill_formed_type);
    else
        tt.append_child(root, n);

    return tt;
}

/// Create a type tree describing the types of the input columns
/// and the output column.
///        
/// @param output_col_num is the column we expect to use as the output
/// (the dependent variable)
///
/// @param ignore_col_nums are a list of column to ignore
///
/// @return type_tree infered
type_tree infer_data_type_tree(const string& fileName,
                               int output_col_num,
                               const vector<int>& ignore_col_nums)
{
    unique_ptr<ifstream> in(open_data_file(fileName));
    string line;
    get_data_line(*in, line);
    if (hasHeader(fileName))
        get_data_line(*in, line);

    OC_ASSERT(1 < line.size(),
        "Error: the data file %s appears to be empty!?\n",
        fileName.c_str());

    type_tree res = infer_row_type_tree(tokenizeRowIO<string>(line,
                                                              output_col_num,
                                                              ignore_col_nums));

    if (!is_well_formed(res)) {
        logger().error() << "Error: Cannot infer type tree for this line: "
            << line << "\nType tree is " <<  res;
    }
    OC_ASSERT(is_well_formed(res),
        "Error: In file %s, cannot deduce data types of some of the "
        "columns in this line=%s\nSee log file for  more info.",
        fileName.c_str(), line.c_str());
    return res;
}

table_tokenizer get_row_tokenizer(string& line)
{
    typedef boost::escaped_list_separator<char> separator;
    typedef boost::tokenizer<separator> tokenizer;
    typedef tokenizer::const_iterator tokenizer_cit;

    // Remove weird symbols at the start of the line (only).
    removeNonASCII(line);
    // Remove carriage return at end of line (for DOS files).
    removeCarriageReturn(line);

    // Tokenize line; current allow tabs, commas, blanks.
    static const separator sep("\\", ",\t ", "\"");
    return tokenizer(line, sep);
}

/// cast string "token" to a vertex of type "tipe"
vertex token_to_vertex(const type_node &tipe, const string& token)
{
    switch (tipe) {

    case id::boolean_type:
        if ("0" == token || "F" == token || "f" == token)
            return id::logical_false;
        else if ("1" == token || "T" == token || "t" == token)
            return id::logical_true;
        else
            OC_ASSERT(false, "Expecting boolean value, got %s", token.c_str());
        break;

    case id::contin_type:
        try {
            return lexical_cast<contin_t>(token);
        } catch(boost::bad_lexical_cast&) {
            OC_ASSERT(false, "Could not cast %s to contin", token.c_str());
        }
        break;

    case id::enum_type:
        // Enum types must begin with an alpha character
        if (isalpha(token[0]))
            return enum_t(token);
        OC_ASSERT(false, "Enum type must begin with alphabetic char, but %s doesn't", token.c_str());
        break;

    case id::definite_object_type:
        return token;
        break;
 
    default:
        stringstream ss;
        ss << "Unable to handle input type=" << tipe << endl;
        OC_ASSERT(0, ss.str().c_str());
    }

    // unreachable
    return id::null_vertex;
}

istream& istreamTable(istream& in, ITable& it, OTable& ot,
                      bool has_header, const type_tree& tt, int pos,
                      const vector<int>& ignore_col_nums)
{
    string line;
    arity_t arity = type_tree_arity(tt);

    if (has_header) {
        get_data_line(in, line);
        pair<vector<string>, string> ioh = tokenizeRowIO<string>(line, pos,
                                                                 ignore_col_nums);
        it.set_labels(ioh.first);
        ot.set_label(ioh.second);
        OC_ASSERT(arity == (arity_t)ioh.first.size(),
                  "ERROR: Input file header/data declaration mismatch: "
                  "The header has %u columns while the first row has "
                  "%d columns.\n",
                  ioh.first.size(), arity);
    }

    // Copy the input types to a vector; we need this to pass as an
    // argument to boost::transform, below.
    vector<type_node> vin_types;   // vector of input types
    transform(get_signature_inputs(tt),
              back_inserter(vin_types), get_type_node);

    // Dependent column type.
    type_node out_type = get_type_node(get_signature_output(tt));

    std::vector<string> lines; 
    while (get_data_line(in, line))
        lines.push_back(line);
    int ls = lines.size();
    it.resize(ls);
    ot.resize(ls);

    // vector of indices [0, lines.size())
    auto ir = boost::irange(0, ls);
    vector<size_t> indices(ir.begin(), ir.end());
    
    auto parse_line = [&](int i) {
        // tokenize the line and fill the input vector and output
        pair<vector<string>, string> io = tokenizeRowIO<string>(lines[i], pos,
                                                                ignore_col_nums);

        // check arity
        OC_ASSERT(arity == (arity_t)io.first.size(),
                  "ERROR: Input file inconsistent: the %uth row has %u "
                  "columns while the first row has %d columns.  All "
                  "rows should have the same number of columns.\n",
                  i + 1, io.first.size(), arity);

        // fill table, based on the types passed in the type-tree
        transform(vin_types, io.first, back_inserter(it[i]), token_to_vertex);
        ot[i] = token_to_vertex(out_type, io.second);
    };
    OMP_ALGO::for_each(indices.begin(), indices.end(), parse_line);
    return in;
}
        
void loadTable(const string& file_name, ITable& it, OTable& ot,
               const type_tree& tt, int pos,
               const vector<int>& ignore_col_nums)
{
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());
    istreamTable(in, it, ot, hasHeader(file_name), tt, pos, ignore_col_nums);
}

Table loadTable(const string& file_name, 
                const std::string& target_feature,
                const std::vector<std::string>& ignore_features)
{
    Table res;
    int target_column = 0;

    // Find the column number of the target feature in the data file,
    // if any.
    if (!target_feature.empty())
        target_column = find_feature_position(file_name, target_feature);

    // Get the list of indexes of features to ignore
    vector<int> ignore_col_nums =
        find_features_positions(file_name, ignore_features);

    ostreamContainer(logger().info() << "Ignore the following columns: ",
                     ignore_col_nums);

    OC_ASSERT(boost::find(ignore_col_nums, target_column)
                  == ignore_col_nums.end(),
                  "You cannot ignore the target feature %s",
                  target_feature.c_str());

    res.tt = infer_data_type_tree(file_name, target_column, ignore_col_nums);
    loadTable(file_name, res.itable, res.otable, res.tt, target_column, ignore_col_nums);
    return res;
}

istream& istreamITable(istream& in, ITable& it,
                       bool has_header, const type_tree& tt,
                       const vector<int>& ignore_col_nums)
{
    string line;
    arity_t arity = type_tree_arity(tt);

    if (has_header) {
        get_data_line(in, line);
        vector<string> h = tokenizeRow<string>(line, ignore_col_nums);
        it.set_labels(h);
        OC_ASSERT(arity == (arity_t)h.size(),
                  "Error: there must be a bug somewhere because the inferred "
                  " arity (%d) doesn't match the number of columns (%u)",
                  arity, h.size());
    }

    // Copy the input types to a vector; we need this to pass as an
    // argument to boost::transform, below.
    vector<type_node> vin_types;   // vector of input types
    transform(get_signature_inputs(tt),
              back_inserter(vin_types), get_type_node);

    // Read all lines at once as it appears to be faster
    std::vector<string> lines; 
    while (get_data_line(in, line))
        lines.push_back(line);
    int ls = lines.size();
    it.resize(ls);

    // vector of indices [0, ls)
    auto ir = boost::irange(0, ls);
    vector<size_t> indices(ir.begin(), ir.end());
    
    auto parse_line = [&](int i) {
        // tokenize the line and fill the input vector and output
        vector<string> vs = tokenizeRow<string>(lines[i], ignore_col_nums);

        // check arity
        OC_ASSERT(arity == (arity_t)vs.size(),
                  "ERROR: Input file inconsistent: the %uth row has %u "
                  "columns while the first row has %d columns.  All "
                  "rows should have the same number of columns.\n",
                  i + 1, vs.size(), arity);

        // fill table, based on the types passed in the type-tree
        transform(vin_types, vs, back_inserter(it[i]), token_to_vertex);
    };
    OMP_ALGO::for_each(indices.begin(), indices.end(), parse_line);
    return in;
}

void loadITable(const string& file_name, ITable& it, const type_tree& tt,
                const vector<int>& ignore_col_nums)
{
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());
    istreamITable(in, it, hasHeader(file_name), tt, ignore_col_nums);
}
    
ITable loadITable(const string& file_name, const vector<int>& ignore_col_nums)
{
    ITable res;
    type_tree tt = infer_data_type_tree(file_name, -1, ignore_col_nums);
    // append an unknown type child at the end for the output
    tt.append_child(tt.begin(), id::unknown_type);
    loadITable(file_name, res, tt, ignore_col_nums);
    return res;
}
        
ostream& ostreamTableHeader(ostream& out, const ITable& it, const OTable& ot,
                            int target_pos)
{
    const auto& ils = it.get_labels();
    const string& ol = ot.get_label();
    int is = ils.size();
    vector<string> header = ils;
    OC_ASSERT(target_pos <= is);
    if (target_pos < 0)
        header.push_back(ol);
    else
        header.insert(header.begin() + target_pos, ol);
    return ostreamContainer(out, header, ",") << endl;
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

ostream& ostreamTable(ostream& out, const ITable& it, const OTable& ot,
                      int target_pos)
{
    // print header
    ostreamTableHeader(out, it, ot, target_pos);
    // print data
    OC_ASSERT(it.size() == ot.size());
    for(size_t row = 0; row < it.size(); ++row) {
        vector<string> content;
        boost::transform(it[row], back_inserter(content), vertex_to_str);
        string oc = vertex_to_str(ot[row]);
        if (target_pos < 0)
            content.push_back(oc);
        else
            content.insert(content.begin() + target_pos, oc);
        ostreamContainer(out, content, ",") << endl;
    }
    return out;
}

ostream& ostreamTable(ostream& out, const Table& table, int target_pos)
{
    return ostreamTable(out, table.itable, table.otable, target_pos);
}

void saveTable(const string& file_name, const ITable& it, const OTable& ot,
               int target_pos)
{
    OC_ASSERT(!file_name.empty(), "No filename specified!");
    ofstream out(file_name.c_str());
    OC_ASSERT(out.is_open(), "Could not open %s", file_name.c_str());
    ostreamTable(out, it, ot, target_pos);
}

void saveTable(const string& file_name, const Table& table, int target_pos)
{
    saveTable(file_name, table.itable, table.otable, target_pos);
}

ostream& ostreamCTableHeader(ostream& out, const CTable& ct)
{
    out << ct.olabel << ",";
    return ostreamlnContainer(out, ct.ilabels, ",");
}

ostream& ostreamCTable(ostream& out, const CTable& ct)
{
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

void subsampleTable(ITable& it, OTable& ot, unsigned nsamples)
{
    OC_ASSERT(it.size() == ot.size());
    if(nsamples < ot.size()) {
        unsigned int nremove = ot.size() - nsamples;
        dorepeat(nremove) {
            unsigned int ridx = randGen().randint(ot.size());
            it.erase(it.begin()+ridx);
            ot.erase(ot.begin()+ridx);
        }
    }
}

void subsampleTable(Table& table, unsigned nsamples)
{
    subsampleTable(table.itable, table.otable, nsamples);
}

void subsampleTable(ITable& it, unsigned nsamples) {
    if(nsamples < it.size()) {
        unsigned int nremove = it.size() - nsamples;
        dorepeat(nremove) {
            unsigned int ridx = randGen().randint(it.size());
            it.erase(it.begin()+ridx);
        }
    }
}

ostream& operator<<(ostream& out, const ITable& it)
{
    ostreamlnContainer(out, it.get_labels(), ",");
    foreach(const vertex_seq& row, it) {
        vector<string> row_str;
        boost::transform(row, back_inserter(row_str), vertex_to_str);
        ostreamlnContainer(out, row_str, ",");
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

ostream& operator<<(ostream& out, const CTable& ct)
{
    return ostreamCTable(out, ct);
}

}} // ~namespaces combo opencog
