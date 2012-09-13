/** table_io.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012 Poulin Holdings
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
 *         Linas Vepstas <linasvepstas@gmail.com>
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

/**
 * Fill the input table only, given a DSV (delimiter-seperated values)
 * file format, where delimiters are ',', ' ' or '\t'.
 *
 * It stuffs all data into the table as strings; type conversion to
 * the appropriate type, and thunking for the header, and ignoring
 * certain features, must alll be done as a separate step.
 */
istream& istreamRawITable(istream& in, ITable& tab)
    throw(AssertionException)
{
    string line;
    std::vector<string> lines;
    while (get_data_line(in, line))
        lines.push_back(line);
    int ls = lines.size();
    tab.resize(ls);

    // Determine the arity from the first line.
    vector<string> fl = tokenizeRow<string>(lines[0]);
    arity_t arity = fl.size();

    auto parse_line = [&](int i) {
        // tokenize the line
        vector<string> io = tokenizeRow<string>(lines[i]);

        // check arity
        if (arity != (arity_t)io.size())
            throw AssertionException(TRACE_INFO,
                  "ERROR: Input file inconsistent: the %uth row has %u "
                  "columns while the first row has %d columns.  All "
                  "rows should have the same number of columns.\n",
                  i + 1, io.size(), arity);

        // fill table with string-valued vertexes.
        foreach (const string& tok, io) {
            vertex v = (vertex) tok;
            tab[i].push_back(v);
        }
    };

    // vector of indices [0, lines.size())
    auto ir = boost::irange(0, ls);
    vector<size_t> indices(ir.begin(), ir.end());
    OMP_ALGO::for_each(indices.begin(), indices.end(), parse_line);
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
 * Infer the column types of the first line of the input table and
 * compare it to the given column types.  If there is a mis-match,
 * then the first row must be a header, i.e. a set of ascii column
 * labels.
 */
bool has_header(ITable& tab, vector<type_node> col_types)
{
    vector<vertex_seq>::const_iterator rowit = tab.begin();

    arity_t arity = (*rowit).size();

    for (arity_t i=0; i<arity; i++) {
        const vertex &v = (*rowit)[i];
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
    istreamRawITable(in, tab);

    // Determine the column types.
    vector<type_node> col_types = infer_column_types(tab);
    tab.set_types(col_types);

    // If there is a header row, then it must be the column labels.
    bool hdr = has_header(tab, col_types);
    if (hdr) {
        vector<vertex> hdr = *(tab.begin());
        vector<string> labels;
        foreach (const vertex& v, hdr) {
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

    foreach (vertex_seq& row, tab)
    {
        // TODO use transform instead of the loop below...
        for (arity_t i=0; i<arity; i++) {
             vertex v = row[i];
             const string& tok = boost::get<string>(v);
             row[i] = token_to_vertex(ig_types[i], tok);
        }
    }

    return in;
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
    type_node targ_type = tab.itable.get_type(target_feature);

    string targ_feat = tab.itable.delete_column(target_feature);

    // If the target feature was emtpy string, then its column zero we are after.
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

// ===========================================================
// ostream regular tables

/// output the header of a data table in CSV format.
ostream& ostreamTableHeader(ostream& out, const ITable& it, const OTable& ot)
{
    vector<string> header = it.get_labels();
    header.insert(header.begin(), ot.get_label());
    return ostreamContainer(out, header, ",") << endl;
}

string vertex_to_str(const vertex& v)
{
    stringstream ss;
    if (is_boolean(v))
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
    for (size_t row = 0; row < it.size(); ++row) {
        vector<string> content;
        boost::transform(it[row], back_inserter(content), vertex_to_str);
        string oc = vertex_to_str(ot[row]);
        // output column is always first.
        content.insert(content.begin(), oc);
        ostreamContainer(out, content, ",") << endl;
    }
    return out;
}

ostream& ostreamTable(ostream& out, const Table& table)
{
    return ostreamTable(out, table.itable, table.otable);
}

void saveTable(const string& file_name, const Table& table)
{
    OC_ASSERT(!file_name.empty(), "No filename specified!");
    ofstream out(file_name.c_str());
    OC_ASSERT(out.is_open(), "Could not open %s", file_name.c_str());
    ostreamTable(out, table.itable, table.otable);
}

// ===========================================================
// ostream CTables

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
    foreach (const vertex_seq& row, it) {
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
    foreach (const vertex& v, ot)
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
