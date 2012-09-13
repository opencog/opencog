/**
 * table_io.h ---
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


#ifndef _OPENCOG_TABLE_IO_H
#define _OPENCOG_TABLE_IO_H

#include <fstream>
#include <string>
#include <vector>

#include <boost/range/algorithm/find.hpp>
#include <boost/tokenizer.hpp>

#include "table.h"
#include "type_tree.h"

namespace opencog { namespace combo {

/**
 * remove the carriage return (for DOS format)
 */
void removeCarriageReturn(std::string& str);

/**
 * remove non ASCII char at the begining of the string
 */
void removeNonASCII(std::string& str);

/**
 * Return true if the next chars in 'in' correspond to carriage return
 * (support UNIX and DOS format) and advance in of the checked chars.
 */
bool checkCarriageReturn(std::istream& in);

/**
 * Check the token, if it is "0" or "1" then it is boolean, otherwise
 * it is contin. It is not 100% reliable of course and should be
 * improved.
 */
type_node infer_type_from_token(const std::string& token);

// used as default for TokenizeRow
static const std::vector<int> empty_int_vec;

/**
 * Take a row, strip away any nnon-ASCII chars and trailing carriage
 * returns, and then return a tokenizer.  Tokenization uses the
 * seperator characters comma, blank, tab (',', ' ' or '\t').
 */
typedef boost::tokenizer<boost::escaped_list_separator<char>> table_tokenizer;
table_tokenizer get_row_tokenizer(std::string& line);

/**
 * Take a line and return a vector containing the elements parsed.
 * Used by istreamTable. This will modify the line to remove leading
 * non-ASCII characters, as well as stripping of any carriage-returns.
 */
template<typename T>
std::vector<T> tokenizeRow(std::string& line,
                           const std::vector<int>& ignore_col_nums = empty_int_vec)
{
    table_tokenizer tok = get_row_tokenizer(line);
    std::vector<T> res;
    int i = 0;
    foreach (const std::string& t, tok) {
        // Record a column, only if it's not to be ignored.
        if (boost::find(ignore_col_nums, i) == ignore_col_nums.end())
            res.push_back(boost::lexical_cast<T>(t));
        i++;
    }
    return res;
}

/**
 * Take a line and return an output and a vector of inputs.
 *
 * The pos variable indicates which token is taken as the output.
 * If pos < 0 then the last token is assumed to be the output.
 * If pos >=0 then that token is used (0 is the first, 1 is the
 * second, etc.)  If pos is out of range, an assert is raised.
 *
 * This will modify the line to remove leading non-ASCII characters,
 * as well as stripping of any carriage-returns.
 */
template<typename T>
std::pair<std::vector<T>, T> tokenizeRowIO(std::string& line,
                                           int pos = 0,
                                           const std::vector<int>& ignore_col_nums
                                           = empty_int_vec)
{
    table_tokenizer tok = get_row_tokenizer(line);
    std::vector<T> inputs;
    T output;
    int i = 0;
    foreach (const std::string& t, tok) {
        // Some input files contain multiple delimiters between columns
        // (e.g. two or three spaces between numeric columns).  This
        // results in empty columns.  Ignore these, and don't i++ either.
        if (0 == t.size()) continue;

        // Record a column, only if it's not to be ignored.
        if (boost::find(ignore_col_nums, i) == ignore_col_nums.end()) {
            if (i != pos)
                inputs.push_back(boost::lexical_cast<T>(t));
            else output = boost::lexical_cast<T>(t);
        }
        i++;
    }
    if (pos < 0) {
        output = inputs.back();
        inputs.pop_back();
    }

    return {inputs, output};
}

//////////////////
// istreamTable //
//////////////////

/// Read a table from an input stream.
std::istream& istreamITable(std::istream& in, ITable& tab,
                           const std::vector<std::string>& ignore_features);

std::istream& istreamTable(std::istream& in, Table& tab,
                           const std::string& target_feature,
                           const std::vector<std::string>& ignore_features);

/**
 * like above but read from file, and return an object Table.
 */
ITable loadITable(const std::string& file_name,
                const std::vector<std::string>& ignore_features);

Table loadTable(const std::string& file_name,
                const std::string& target_feature,
                const std::vector<std::string>& ignore_features);

Table loadTable(const std::string& file_name,
                const type_tree& signature,
                const std::string& target_feature,
                const std::vector<std::string>& ignore_features);


//////////////////
// ostreamTable //
//////////////////

/// output a data table in CSV format. Boolean values are output in
/// binary form (0 for false, 1 for true).
std::ostream& ostreamTable(std::ostream& out,
                           const ITable& it, const OTable& ot);

/// like above but take a table instead of an input and output table
std::ostream& ostreamTable(std::ostream& out, const Table& table);

/// like above but take a table instead of a input and output table
void saveTable(const std::string& file_name, const Table& table);

/// output a compressed table in pseudo CSV format
std::ostream& ostreamCTable(std::ostream& out, const CTable& ct);

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

std::ifstream* open_data_file(const std::string& fileName);

std::ostream& operator<<(std::ostream& out, const ITable& it);

std::ostream& operator<<(std::ostream& out, const OTable& ot);

std::ostream& operator<<(std::ostream& out, const CTable& ct);

std::ostream& operator<<(std::ostream& out, const complete_truth_table& tt);

}} // ~namespaces combo opencog

#endif // _OPENCOG_TABLE_IO_H
