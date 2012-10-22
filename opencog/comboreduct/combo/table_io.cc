/** table_io.cc ---
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
#include <atomic>
#include <iomanip>

#include <boost/algorithm/string.hpp>
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

// -------------------------------------------------------
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

/// Get one line of actual data.
/// This ignores lines that start with a 'standard comment char'
///
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

        // Remove weird symbols at the start of the line (only).
        removeNonASCII(line);
        // Remove carriage return at end of line (for DOS files).
        removeCarriageReturn(line);

        return is;
    }
}

// -------------------------------------------------------

table_tokenizer get_row_tokenizer(const string& line)
{
    typedef boost::escaped_list_separator<char> separator;
    typedef boost::tokenizer<separator> tokenizer;

    // Tokenize line; currently, we allow tabs, commas, blanks.
    static const separator sep("\\", ",\t ", "\"");
    return tokenizer(line, sep);
}

// Same as above, but only allow commas as a column separator.
table_tokenizer get_sparse_row_tokenizer(const string& line)
{
    typedef boost::escaped_list_separator<char> separator;
    typedef boost::tokenizer<separator> tokenizer;

    // Tokenize line; currently, we allow tabs, commas, blanks.
    static const separator sep("\\", ",", "\"");
    return tokenizer(line, sep);
}

/**
 * Take a line and return a vector containing the elements parsed.
 * Used by istreamTable. This will modify the line to remove leading
 * non-ASCII characters, as well as stripping of any carriage-returns.
 */
vector<string> tokenizeSparseRow(const string& line)
{
    table_tokenizer tok = get_sparse_row_tokenizer(line);
    vector<string> res;
    for (string t : tok) {
        boost::trim(t);
        res.push_back(t);
    }
    return res;
}

// -------------------------------------------------------
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
 * Given an input string, guess the type of the string.
 * Inferable types are: boolean, contin and enum.
 * Compare this to 'curr_guess', and upgrade the type inference
 * if it can be done consistently.
 */
type_node infer_type_from_token(type_node curr_guess, const string& token)
{
    type_node tokt = infer_type_from_token(token);

    // First time, just go with the flow.
    if (id::unknown_type == curr_guess)
        return tokt;

    // Yayy! its consistent!
    if (tokt == curr_guess)
        return tokt;

    // If we saw 0,1 when expecting a contin, its a contin.
    if ((id::contin_type == curr_guess) && (id::boolean_type == tokt))
        return curr_guess;

    // If we thought its a boolean 0,1 it might be a contin.
    if ((id::boolean_type == curr_guess) && (id::contin_type == tokt))
        return tokt;

    // If we got to here, then there's some sort of unexpected
    // inconsistency in the column types; we've got to presume that
    // its just some crazy ascii string, i.e. enum_type.
    return id::enum_type;
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

// ===========================================================
// istream regular tables.

const char *sparse_delim = " : ";

/**
 * Fill the input table, given a file in DSV (delimiter-seperated values)
 * format.  The delimiters are ',', ' ' or '\t'.
 *
 * It stuffs all data into the table as strings; type conversion to
 * the appropriate type, and thunking for the header, and ignoring
 * certain features, must all be done as a separate step.
 */
istream& istreamRawITable(istream& in, ITable& tab)
    throw(std::exception, AssertionException)
{
    streampos beg = in.tellg();

    // Get the entire dataset into memory
    string line;
    std::vector<string> lines;

    // Read first few by hand.
    // The first might be labels, so we must get the second line.
    dorepeat(2) {
        get_data_line(in, line);
        lines.push_back(line);
    }

    // If it is a sparse file, we are outta here.
    // Throw an std::exception, since we don't want to log this as an
    // error (all the other exception types log to the log file).
    if (string::npos != line.find (sparse_delim)) {
        in.seekg(beg);
        throw std::exception();
    }

    // Grab the rest of the file.
    while (get_data_line(in, line))
        lines.push_back(line);

    // Determine the arity from the first line.
    vector<string> fl = tokenizeRow<string>(lines[0]);
    arity_t arity = fl.size();

    atomic<int> arity_fail_row(-1);
    auto parse_line = [&](int i)
    {
        // tokenize the line
        vector<string> io = tokenizeRow<string>(lines[i]);

        // Check arity
        if (arity != (arity_t)io.size())
            arity_fail_row = i + 1;
        
        // Fill table with string-valued vertexes.
        for (const string& tok : io) {
            vertex v(tok);
            tab[i].push_back(v);
        }
    };

    // Vector of indices [0, lines.size())
    size_t ls = lines.size();
    tab.resize(ls);
    auto ir = boost::irange((size_t)0, ls);
    vector<size_t> indices(ir.begin(), ir.end());
    OMP_ALGO::for_each(indices.begin(), indices.end(), parse_line);

    if (-1 != arity_fail_row) {
        in.seekg(beg);
        OC_ASSERT(false,
                  "ERROR: Input file inconsistent: the %uth row has "
                  "a different number of columns than the rest of the file.  "
                  "All rows should have the same number of columns.\n",
                  arity_fail_row.load());
    }
    return in;
}

// like above but ignore indices
// XXX The code below is very nearly a cut-n-pste of above; lets not
// do that, and instead simply provide a default argument for ignored indexes!
// cut-n-paste == badness
istream& istreamRawITable_ignore_indices(istream& in, ITable& tab,
                                         const vector<unsigned>& ignored_indices)
    throw(std::exception, AssertionException)
{
    streampos beg = in.tellg();

    // Get the entire dataset into memory
    string line;
    std::vector<string> lines;

    // Read first few by hand.
    // The first might be labels, so we must get the second line.
    dorepeat(2) {
        get_data_line(in, line);
        lines.push_back(line);
    }

    // If it is a sparse file, we are outta here.
    // Throw an std::exception, since we don't want to log this as an
    // error (all the other exception types log to the log file).
    if (string::npos != line.find (sparse_delim)) {
        in.seekg(beg);
        throw std::exception();
    }

    // Grab the rest of the file.
    while (get_data_line(in, line))
        lines.push_back(line);

    // Determine the arity from the first line.
    vector<string> fl = tokenizeRow_ignore_indices<string>(lines[0],
                                                           ignored_indices);
    arity_t arity = fl.size();

    atomic<int> arity_fail_row(-1);
    auto parse_line = [&](int i)
    {
        // tokenize the line
        vector<string> io = tokenizeRow_ignore_indices<string>(lines[i],
                                                               ignored_indices);

        // Check arity
        if (arity != (arity_t)io.size())
            arity_fail_row = i + 1;
        
        // Fill table with string-valued vertexes.
        for (const string& tok : io) {
            vertex v(tok);
            tab[i].push_back(v);
        }
    };

    // Vector of indices [0, lines.size())
    size_t ls = lines.size();
    tab.resize(ls);
    auto ir = boost::irange((size_t)0, ls);
    vector<size_t> indices(ir.begin(), ir.end());
    OMP_ALGO::for_each(indices.begin(), indices.end(), parse_line);

    if (-1 != arity_fail_row) {
        in.seekg(beg);
        OC_ASSERT(false,
                  "ERROR: Input file inconsistent: the %uth row has "
                  "a different number of columns than the rest of the file.  "
                  "All rows should have the same number of columns.\n",
                  arity_fail_row.load());
    }
    return in;
}

vector<string> get_header(const string& file_name)
{
    ifstream in(file_name.c_str());
    string line;
    get_data_line(in, line);
    return tokenizeRow<string>(line);
}

/**
 * Fill the input table, given a file in 'sparse' format.
 *
 * The sparse table format consists of some fixed number of columns,
 * in comma-separated format, followed by key-value pairs, also
 * tab-separated. viz:
 * 
 *     val, val, val, name:val, name:val, name:val
 * 
 * Thus, for example, a row such as 
 * 
 *    earn, issued : 1, results : 2, ending : 1, including : 1
 * 
 * indicates that there one fixed column, of enum type, (the enum value
 * being "earn"), and that features called "issued", "ending" and 
 * "including" have a contin value of 1.0  and "results" has a contin
 * value of 2.
 * 
 * The routine does NOT store the table in sparse format: it stores the
 * full, exploded table. This could be bad ...
 * TODO: we really need a sparse table format, as well.  
 *
 * The "Raw" format has all data as strings; type conversion to the
 * appropriate type, must all be done as a separate step.
 */
istream& istreamSparseITable(istream& in, ITable& tab)
{
    // The raw dataset
    std::vector<string> lines;

    // The first non-comment line is assumed to be the header.
    // ... unless it isn't. (The header must not contain a colon).
    vector<string> labs;
    size_t fixed_arity = 0;
    string header;
    get_data_line(in, header);
    if (string::npos == header.find(sparse_delim)) {
        // Determine the arity of the fixed columns
        vector<string> hdr = tokenizeSparseRow(header);
        fixed_arity = hdr.size();
        labs = hdr;
    }
    else {
        lines.push_back(header);
    }

    // Get the entire dataset into memory
    string iline;
    while (get_data_line(in, iline))
        lines.push_back(iline);

    if (0 == fixed_arity) {
        vector<string> fixy = tokenizeSparseRow(lines[0]);
        // count commas, until a semi-colon is found.
        while (string::npos == fixy[fixed_arity].find(sparse_delim)) 
            fixed_arity++;
    }
    logger().info() << "Sparse file fixed column count="<<fixed_arity;

    // Get a list of all of the features.
    set<string> feats;
    // All sparse features have the same type.
    type_node feat_type = id::unknown_type;

    // Fixed features may have different types, by column.
    vector<type_node> types;
    types.resize(fixed_arity);
    for (size_t off = 0; off < fixed_arity; off++) 
        types[off] = id::unknown_type;

    size_t d_len = strlen(sparse_delim);

    for (const string& line : lines) {
        vector<string> chunks = tokenizeSparseRow(line);
        vector<string>::const_iterator pit = chunks.begin();

        // Infer the types of the fixed features.
        size_t off = 0;
        for (; off < fixed_arity; off++, pit++) 
            types[off] = infer_type_from_token(types[off], *pit);

        for (; pit != chunks.end(); pit++) {

            // Rip out the key-value pairs
            size_t pos = pit->find(sparse_delim);
            if (string::npos == pos)
                break;
            string key = pit->substr(0, pos);
            boost::trim(key);
            string val = pit->substr(pos + d_len);
            boost::trim(val);

            // Store the key, uniquely.  Store best guess as the type.
            feats.insert(key);
            feat_type = infer_type_from_token(feat_type, val);
        }
    }
    logger().info() << "Sparse file unique features count=" << feats.size();
    logger().info() << "Sparse file feature type=" << feat_type;
    logger().info() << "Sparse file row count=" << lines.size();

    // Convert the feature set into a list of labels.
    // 'index' is a map from feature name to column number.
    size_t cnt = fixed_arity;
    map<const string, size_t> index;
    for (const string& key : feats) {
        types.push_back(feat_type);
        labs.push_back(key);
        index[key] = cnt;
        cnt++;
    }
    tab.set_labels(labs);
    tab.set_types(types);

    // And finally, stuff up the table.
    // The function below tokenizes one row, and jams it into the table
    size_t row_len = labs.size();
    auto fill_line = [&](int i)
    {
        const string& line = lines[i];
        vector<vertex> row;
        row.resize(row_len);

        // Tokenize the line
        vector<string> chunks = tokenizeSparseRow(line);
        vector<string>::const_iterator pit = chunks.begin();

        // First, handle the fixed columns
        // Cast them to the appropriate types
        size_t off = 0;
        for (; off < fixed_arity; off++, pit++) {
            row[off] = token_to_vertex(types[off], *pit);
        }

        // Next, handle the key-value pairs, again, casting
        // them to the appropriate types.
        for (; pit != chunks.end(); pit++) {
            size_t pos = pit->find(sparse_delim);
            if (string::npos == pos)
                break;
            string key = pit->substr(0, pos);
            boost::trim(key);
            string val = pit->substr(pos + d_len);
            boost::trim(val);
            off = index[key];
            row[off] = token_to_vertex(feat_type, val);
        }
        tab[i] = row;
    };

    // Vector of indices [0, lines.size())
    size_t ls = lines.size();
    tab.resize(ls);
    auto ir = boost::irange((size_t)0, ls);
    vector<size_t> indices(ir.begin(), ir.end());
    OMP_ALGO::for_each(indices.begin(), indices.end(), fill_line);

    return in;
}

/**
 * Infer the column types of the input table
 */
vector<type_node> infer_column_types(const ITable& tab)
{
    vector<vertex_seq>::const_iterator rowit = tab.begin();

    vector<type_node> types;
    arity_t arity = (*rowit).size();
    types.resize(arity);
    for (arity_t i=0; i<arity; i++) {
        types[i] = id::unknown_type;
    }

    // Skip the first line, it might be a header...
    // and that would confuse type inference.
    if (tab.size() > 1)
        rowit++;
    for (; rowit != tab.end(); rowit++)
    {
        // TODO: could use a two-arg transform, here, but its tricky ...
        for (arity_t i=0; i<arity; i++) {
            const vertex &v = (*rowit)[i];
            const string& tok = boost::get<string>(v);
            types[i] = infer_type_from_token(types[i], tok);
        }
    }
    return types;
}

/**
 * Infer the column types of the first line of a raw input table and
 * compare it to the given column types.  If there is a mis-match,
 * then the first row must be a header, i.e. a set of ascii column
 * labels.
 */
bool has_header(ITable& tab, vector<type_node> col_types)
{
    const vertex_seq& row = *tab.begin();

    arity_t arity = row.size();

    for (arity_t i=0; i<arity; i++) {
        const vertex& v = row[i];
        const string& tok = boost::get<string>(v);
        type_node flt = infer_type_from_token(col_types[i], tok);
        if ((id::enum_type == flt) && (id::enum_type != col_types[i]))
            return true;
    }
    return false;
}

/**
 * Fill the input table only, given a DSV (delimiter-seperated values)
 * file format, where delimiters are ',', ' ' or '\t'.
 *
 * This algorithm makes several passes over the data.  First, it reads
 * the entire table, as a collection of strings.  Next, it tries to
 * infer the column types, and the presence of a header.
 */
istream& istreamITable(istream& in, ITable& tab,
                       const vector<string>& ignore_features)
{
    try {
        istreamRawITable(in, tab);
    }
    catch (std::exception e) {
        istreamSparseITable(in, tab);
        // Get rid of the unwanted columns.
        tab.delete_columns(ignore_features);
        return in;
    }

    // Determine the column types.
    vector<type_node> col_types = infer_column_types(tab);
    tab.set_types(col_types);

    // If there is a header row, then it must be the column labels.
    bool hdr = has_header(tab, col_types);
    if (hdr) {
        vector<vertex> hdr = *(tab.begin());
        vector<string> labels;
        for (const vertex& v : hdr) {
            const string& tok = boost::get<string>(v);
            labels.push_back(tok);
        }
        tab.set_labels(labels);
        tab.erase(tab.begin());
    }

    // Now that we have some column labels to work off of,
    // Get rid of the unwanted columns.
    tab.delete_columns(ignore_features);

    // Finally, perform a column type conversion
    arity_t arity = tab.get_arity();
    const vector<type_node>& ig_types = tab.get_types();

    OMP_ALGO::for_each (tab.begin(), tab.end(),
        [&](vertex_seq& row) {
            for (arity_t i=0; i<arity; i++) {
                 const string& tok = boost::get<string>(row[i]);
                 row[i] = token_to_vertex(ig_types[i], tok);
            }
        });
    
    return in;
}

/**
 * Like istreamITable but add the option to ignore indices.
 *
 * It's akind of a temporary hack, till it's clear that this is much
 * faster and we should recode istreamITable to ignore features
 * head-on.
 *
 * Also, it assumes that the dataset is not sparse.
 */
istream& istreamITable_ignore_indices(istream& in, ITable& tab,
                                      const vector<unsigned>& ignore_indices)
{
    istreamRawITable_ignore_indices(in, tab, ignore_indices);

    // Determine the column types.
    vector<type_node> col_types = infer_column_types(tab);
    tab.set_types(col_types);

    // If there is a header row, then it must be the column labels.
    bool hdr = has_header(tab, col_types);
    if (hdr) {
        vector<vertex> hdr = *(tab.begin());
        vector<string> labels;
        for (const vertex& v : hdr) {
            const string& tok = boost::get<string>(v);
            labels.push_back(tok);
        }
        tab.set_labels(labels);
        tab.erase(tab.begin());
    }

    // Finally, perform a column type conversion
    arity_t arity = tab.get_arity();
    const vector<type_node>& ig_types = tab.get_types();

    OMP_ALGO::for_each (tab.begin(), tab.end(),
        [&](vertex_seq& row) {
            for (arity_t i=0; i<arity; i++) {
                 const string& tok = boost::get<string>(row[i]);
                 row[i] = token_to_vertex(ig_types[i], tok);
            }
        });
    
    return in;
}

OTable loadOTable(const string& file_name, const string& target_feature)
{
    vector<string> ignore_features;
    for (const string& l : get_header(file_name))
        if (l != target_feature)
            ignore_features.push_back(l);

    ITable itab = loadITable(file_name, ignore_features);
    OTable res(itab.get_column_data(target_feature), target_feature);
    return res;
}

ITable loadITable(const string& file_name,
                  const vector<string>& ignore_features)
{
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());

    ITable res;
    istreamITable(in, res, ignore_features);
    return res;
}

/**
 * Like loadITable but it is optimized by ignoring features head-on
 * (rather than loading them, then removing them.
 *
 * WARNING: it assumes the dataset has a header!!!
 */
ITable loadITable_optimized(const string& file_name,
                            const vector<string>& ignore_features)
{
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());

    // determined ignore_indices
    vector<unsigned> ignore_indices = get_indices(ignore_features,
                                                  get_header(file_name));
    
    ITable res;
    istreamITable_ignore_indices(in, res, ignore_indices);
    return res;
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
istream& istreamTable(istream& in, Table& tab,
                      const string& target_feature,
                      const vector<string>& ignore_features)
{
    istreamITable(in, tab.itable, ignore_features);

    tab.otable = tab.itable.get_column_data(target_feature);
    OC_ASSERT(0 != tab.otable.size(), 
              "Fatal Error: target feature \"%s\" not found",
              target_feature.c_str());

    tab.target_pos = tab.itable.get_column_offset(target_feature);
    if (tab.target_pos == tab.get_arity() - 1)
        tab.target_pos = -1;    // the last position is -1
    
    type_node targ_type = tab.itable.get_type(target_feature);

    string targ_feat = tab.itable.delete_column(target_feature);

    tab.otable.set_label(targ_feat);
    tab.otable.set_type(targ_type);

    // Record the table signature.
    tab.tt = gen_signature(tab.itable.get_types(), targ_type);

    return in;
}

/**
 * Like istreamTable but optimize by ignoring features head-on rather
 * than loading them then removing them.
 *
 * Warning: only works on dense data with header file.
 */
istream& istreamTable_ignore_indices(istream& in, Table& tab,
                                     const string& target_feature,
                                     const vector<unsigned>& ignore_indices)
{    
    istreamITable_ignore_indices(in, tab.itable, ignore_indices);

    tab.otable = tab.itable.get_column_data(target_feature);
    OC_ASSERT(0 != tab.otable.size(), 
              "Fatal Error: target feature \"%s\" not found",
              target_feature.c_str());

    tab.target_pos = tab.itable.get_column_offset(target_feature);
    if (tab.target_pos == tab.get_arity() - 1)
        tab.target_pos = -1;    // the last position is -1
    
    type_node targ_type = tab.itable.get_type(target_feature);

    string targ_feat = tab.itable.delete_column(target_feature);

    tab.otable.set_label(targ_feat);
    tab.otable.set_type(targ_type);

    // Record the table signature.
    tab.tt = gen_signature(tab.itable.get_types(), targ_type);

    return in;
}

/**
 * istream. If the file name is not correct then an OC_ASSERT is
 * raised.
 */
Table loadTable(const string& file_name,
                const string& target_feature,
                const vector<string>& ignore_features)
{
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());

    Table res;
    istreamTable(in, res, target_feature, ignore_features);
    return res;
}

/**
 * Like loadTable but ignore the features head on instead of loading
 * then removing them.
 *
 * Warning: only works on dense datasets with header.
 */
Table loadTable_optimized(const string& file_name,
                          const string& target_feature,
                          const vector<string>& ignore_features)
{
    OC_ASSERT(!file_name.empty(), "the file name is empty");
    ifstream in(file_name.c_str());
    OC_ASSERT(in.is_open(), "Could not open %s", file_name.c_str());

    // determined ignore_indices
    vector<unsigned> ignore_indices = get_indices(ignore_features,
                                                  get_header(file_name));

    Table res;
    istreamTable_ignore_indices(in, res, target_feature, ignore_indices);
    return res;
}

// ===========================================================
// ostream regular tables

string vertex_to_str(const vertex& v)
{
    stringstream ss;
    if (is_boolean(v))
        ss << vertex_to_bool(v);
    else
        ss << v;
    return ss.str();
}

void saveTable(const string& file_name, const Table& table)
{
    OC_ASSERT(!file_name.empty(), "No filename specified!");
    ofstream out(file_name.c_str());
    OC_ASSERT(out.is_open(), "Could not open %s", file_name.c_str());
    ostreamTable(out, table);
}

// ===========================================================
// ostream CTables

ostream& ostreamCTableHeader(ostream& out, const CTable& ct)
{
    return ostreamlnContainer(out, ct.get_labels(), ",");
}

ostream& ostreamCTable(ostream& out, const CTable& ct)
{
    // print header
    ostreamCTableHeader(out, ct);
    // print data
    for (const auto& v : ct) {
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

// ===========================================================
// subsample stuff

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

// ===========================================================
// operator<< for the various tables and stuff.

ostream& operator<<(ostream& out, const ITable& it)
{
    ostreamlnContainer(out, it.get_labels(), ",");
    ostreamlnContainer(out, it.get_types(), ",");
    for (const vertex_seq& row : it) {
        vector<string> row_str;
        boost::transform(row, back_inserter(row_str), vertex_to_str);
        ostreamlnContainer(out, row_str, ",");
    }
    return out;
}

ostream& operator<<(ostream& out, const OTable& ot)
{
    if (!ot.get_label().empty())
        out << ot.get_label() << endl;
    out << ot.get_type() << endl;
    for (const vertex& v : ot)
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
