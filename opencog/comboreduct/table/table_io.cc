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
#include <boost/variant.hpp>

#include <opencog/util/dorepeat.h>
#include <opencog/util/oc_omp.h>
#include <opencog/util/comprehension.h>

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

pair<string, string> parse_key_val(string chunk) {
    pair<string, string> res;
    size_t pos = chunk.find(sparse_delim);
    if (string::npos == pos)
        return res;
    string key = chunk.substr(0, pos);
    boost::trim(key);
    string val = chunk.substr(pos + strlen(sparse_delim));
    boost::trim(val);
    return {key, val};
}
        
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
builtin token_to_boolean(const string& token)
{
    if ("0" == token || "F" == token || "f" == token)
        return id::logical_false;
    else if ("1" == token || "T" == token || "t" == token)
        return id::logical_true;
    else {
        OC_ASSERT(false, "Expecting boolean value, got %s", token.c_str());
        return builtin();
    }
}
contin_t token_to_contin(const string& token)
{
    try {
        return lexical_cast<contin_t>(token);
    } catch(boost::bad_lexical_cast&) {
        OC_ASSERT(false, "Could not cast %s to contin", token.c_str());
        return contin_t();
    }
}
vertex token_to_vertex(const type_node &tipe, const string& token)
{
    switch (tipe) {

    case id::boolean_type:
        return token_to_boolean(token);

    case id::contin_type:
        return token_to_contin(token);

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

/**
 * Fill the input table, given a file in DSV (delimiter-seperated values)
 * format.  The delimiters are ',', ' ' or '\t'.
 *
 * It stuffs all data into the table as strings; type conversion to
 * the appropriate type, and thunking for the header, and ignoring
 * certain features, must all be done as a separate step.
 */
istream& istreamRawITable(istream& in, ITable& tab,
                          const vector<unsigned>& ignored_indices)
    throw(std::exception, AssertionException)
{
    streampos beg = in.tellg();

    // Get the entire dataset into memory
    string line;
    std::vector<string> lines;

    // Read first few by hand. The first might be labels, so we must
    // get at least the second line. But the second line might have
    // all default feature values (i.e. no colon), so get the third...
    dorepeat(20) {
        if (!get_data_line(in, line))
            break;
        // If it is a sparse file, we are outta here.
        // Throw an std::exception, since we don't want to log this as an
        // error (all the other exception types log to the log file).
        if (string::npos != line.find (sparse_delim)) {
            in.seekg(beg);
            throw std::exception();
        }
        lines.push_back(line);
    }

    // Grab the rest of the file.
    while (get_data_line(in, line))
        lines.push_back(line);

    // Determine the arity from the first line.
    vector<string> fl = tokenizeRow<string>(lines[0], ignored_indices);
    arity_t arity = fl.size();

    atomic<int> arity_fail_row(-1);
    auto parse_line = [&](int i)
    {
        // tokenize the line and fill the table with
        tab[i] = tokenizeRow<string>(lines[i], ignored_indices);

        // Check arity
        if (arity != (arity_t)tab[i].size())
            arity_fail_row = i + 1;
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
    logger().info() << "Sparse file fixed column count=" << fixed_arity;

    // Get a list of all of the features.
    set<string> feats;
    // All sparse features have the same type.
    type_node feat_type = id::unknown_type;

    // Fixed features may have different types, by column.
    vector<type_node> types(fixed_arity, id::unknown_type);

    for (const string& line : lines) {
        vector<string> chunks = tokenizeSparseRow(line);
        vector<string>::const_iterator pit = chunks.begin();

        // Infer the types of the fixed features.
        size_t off = 0;
        for (; off < fixed_arity; off++, pit++) 
            types[off] = infer_type_from_token(types[off], *pit);

        for (; pit != chunks.end(); pit++) {
            // Rip out the key-value pairs
            auto key_val = parse_key_val(*pit);
            if (key_val == pair<string, string>())
                break;
            // Store the key, uniquely.  Store best guess as the type.
            feats.insert(key_val.first);
            feat_type = infer_type_from_token(feat_type, key_val.second);
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
    from_sparse_tokens_visitor fstv(types, index, fixed_arity);
    auto fill_line = [&](int i)
    {
        const string& line = lines[i];
        // Tokenize the line
        vector<string> chunks = tokenizeSparseRow(line);
        multi_type_seq row = fstv(chunks);
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
 * Infer the column types of the input table. It is assumed the
 * table's rows are vector of strings.
 */
vector<type_node> infer_column_types(const ITable& tab)
{
    vector<multi_type_seq>::const_iterator rowit = tab.begin();

    arity_t arity = rowit->size();
    vector<type_node> types(arity, id::unknown_type);

    // Skip the first line, it might be a header...
    // and that would confuse type inference.
    if (tab.size() > 1)
        rowit++;
    for (; rowit != tab.end(); rowit++)
    {
        const string_seq& tokens = rowit->get_seq<string>();
        for (arity_t i=0; i<arity; i++)
            types[i] = infer_type_from_token(types[i], tokens[i]);
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
    const string_seq& row = tab.begin()->get_seq<string>();

    arity_t arity = row.size();

    for (arity_t i=0; i<arity; i++) {
        type_node flt = infer_type_from_token(col_types[i], row[i]);
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
    if (has_header(tab, col_types)) {
        tab.set_labels(tab.begin()->get_seq<string>());
        tab.erase(tab.begin());
    }

    // Now that we have some column labels to work off of,
    // Get rid of the unwanted columns.
    tab.delete_columns(ignore_features);

    // Finally, perform a column type conversion
    from_tokens_visitor ftv(tab.get_types());
    auto aft = apply_visitor(ftv);
    OMP_ALGO::transform(tab.begin(), tab.end(), tab.begin(),
                        [&](multi_type_seq& seq) {
                            return aft(seq.get_variant());
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
    istreamRawITable(in, tab, ignore_indices);

    // Determine the column types.
    vector<type_node> col_types = infer_column_types(tab);
    tab.set_types(col_types);

    // If there is a header row, then it must be the column labels.
    if (has_header(tab, col_types)) {
        tab.set_labels(tab.begin()->get_seq<string>());
        tab.erase(tab.begin());
    }

    // Finally, perform a column type conversion
    from_tokens_visitor ftv(tab.get_types());
    auto aft = apply_visitor(ftv);
    OMP_ALGO::transform(tab.begin(), tab.end(), tab.begin(),
                        [&](multi_type_seq& seq) {
                            return aft(seq.get_variant());
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

// Parse a CTable row
CTable::value_type parseCTableRow(const type_tree& tt, const std::string& row_str)
{
    // split the string between input and output
    unsigned end_outputs_pos = row_str.find("}");
    string outputs = row_str.substr(1, end_outputs_pos - 1),
        inputs = row_str.substr(end_outputs_pos + 2); // +2 to go
                                                      // passed the
                                                      // following ,

    // convert the inputs string into multi_type_seq
    type_node_seq tns = vector_comp(get_signature_inputs(tt), get_type_node);
    vector<string> input_seq = tokenizeRow<string>(inputs);
    from_tokens_visitor ftv(tns);
    multi_type_seq input_values = ftv(input_seq);

    // convert the outputs string into CTable::counter_t
    vector<string> output_pair_seq  = tokenizeRow<string>(outputs);
    CTable::counter_t counter;
    for (const string& pair_str : output_pair_seq) {
        unsigned sep_pos = pair_str.find(":");
        string key_str = pair_str.substr(0, sep_pos),
            value_str = pair_str.substr(sep_pos + 1);
        vertex v = token_to_vertex(get_type_node(get_signature_output(tt)),
                                   key_str);
        unsigned count = stoi(value_str);
        counter[v] = count;
    }
    return CTable::value_type(input_values, counter);
}

// WARNING: this implementation only supports boolean ctable!!!!
std::istream& istreamCTable(std::istream& in, CTable& ctable)
{
    ////////////////
    // set header //
    ////////////////
    string header_line;
    get_data_line(in, header_line);
    auto labels = tokenizeRow<string>(header_line);
    ctable.set_labels(labels);

    ////////////////////////
    // set type signature //
    ////////////////////////
    // HACK THIS PART TO MAKE IT SUPPORT OTHER TYPES THAN BOOLEAN
    ctable.set_signature(gen_signature(id::boolean_type, ctable.get_arity()));

    /////////////////
    // set content //
    /////////////////
    std::vector<string> lines;
    // read the entire file
    {
        string line;
        while (get_data_line(in, line))
            lines.push_back(line);
    }
    // parse each line and fill the ctable
    for (const string& line : lines)
        ctable.insert(parseCTableRow(ctable.get_signature(), line));

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

CTable loadCTable(const string& file_name)
{
    CTable ctable;
    OC_ASSERT(!file_name.empty(), "No filename specified!");
    ifstream in(file_name.c_str());
    istreamCTable(in, ctable);
    return ctable;
}

// ===========================================================
// ostream regular tables

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

ostream& ostreamCTableRow(ostream& out, const CTable::value_type& ctv)
{
    to_strings_visitor tsv;
    auto ats = boost::apply_visitor(tsv);
    // print map of outputs
    out << "{";
    for(auto it = ctv.second.begin(); it != ctv.second.end();) {
        out << table_fmt_vertex_to_str(it->first) << ":" << it->second;
        if(++it != ctv.second.end())
            out << ",";
    }
    out << "},";
    // print inputs
    return ostreamlnContainer(out, ats(ctv.first.get_variant()), ",");
}

ostream& ostreamCTable(ostream& out, const CTable& ct)
{
    // print header
    ostreamCTableHeader(out, ct);
    // print data
    for (const auto& v : ct)
        ostreamCTableRow(out, v);

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
    to_strings_visitor tsv;
    for (const auto& row : it) {
        vector<string> row_str = boost::apply_visitor(tsv, row.get_variant());
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
        out << table_fmt_vertex_to_str(v) << endl;
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
