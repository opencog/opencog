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

#include "table.h"
#include "type_tree.h"

namespace opencog { namespace combo {

// using boost::variant;
// using boost::adaptors::map_values;

//////////////////
// istreamTable //
//////////////////

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
 * Return the arity of the table provided in istream (by counting the
 * number of elements of the first line).
 */
arity_t istreamArity(std::istream& in);
/**
 * Helper, like above but given the file name instead of istream
 */
arity_t dataFileArity(const std::string& dataFileName);

std::vector<std::string> loadHeader(const std::string& file_name);

/**
 * check if the data file has a header. That is whether the first row
 * starts with a sequence of output and input labels
 */
bool hasHeader(const std::string& dataFileName);

/**
 * Check the token, if it is "0" or "1" then it is boolean, otherwise
 * it is contin. It is not 100% reliable of course and should be
 * improved.
 */
type_node infer_type_from_token(const std::string& token);

/**
 * take a row in input as a pair {inputs, output} and return the type
 * tree corresponding to the function mapping inputs to output. If the
 * inference fails then it returns a type_tree with
 * id::ill_formed_type as root.
 */
type_tree infer_row_type_tree(std::pair<std::vector<std::string>,
                                        std::string>& row);

// used as default of infer_data_type_tree and other IO function
static const std::vector<int> empty_int_vec;

/// Create a type tree describing the types of the input columns
/// and the output column.
///        
/// @param output_col_num is the column we expect to use as the output
/// (the dependent variable)
///
/// @param ignore_col_nums are a list of column to ignore
///
/// @return type_tree infered
type_tree infer_data_type_tree(const std::string& fileName,
                               int output_col_num = 0,
                               const std::vector<int>& ignore_col_nums
                               = empty_int_vec);

/**
 * Find the column numbers associated with the names features
 * 
 * If the target begins with an alpha character, it is assumed to be a
 * column label. We return the column number; 0 is the left-most column.
 *
 * If the target is numeric, just assum that it is a column number.
 */
std::vector<int> find_features_positions(const std::string& fileName,
                                         const std::vector<std::string>& features);
int find_feature_position(const std::string& fileName,
                          const std::string& feature);

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

/**
 * Fill an input table give an istream of DSV file format, where
 * delimiters are ',',' ' or '\t'. All columns, except the
 * ignore_col_nums ones are considered for the ITable.
 */
std::istream& istreamITable(std::istream& in, ITable& it,
                            bool has_header, const type_tree& tt,
                            const std::vector<int>& ignore_col_nums = empty_int_vec);

/**
 * Like above but takes a file_name instead of a istream and
 * automatically infer whether it has header.
 */
void loadITable(const std::string& file_name, ITable& it, const type_tree& tt,
                const std::vector<int>& ignore_col_nums = empty_int_vec);

/**
 * Like above but return an ITable and automatically infer the tree
 * tree.
 */
ITable loadITable(const std::string& file_name,
                  const std::vector<int>& ignore_col_nums = empty_int_vec);

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
std::istream& istreamTable(std::istream& in, ITable& it, OTable& ot,
                           bool has_header, const type_tree& tt, int pos = 0,
                           const std::vector<int>& ignore_col_nums
                           = empty_int_vec);

/**
 * like istreamTable above but take an string (file name) instead of
 * istream. If the file name is not correct then an OC_ASSERT is
 * raised.
 */
void loadTable(const std::string& file_name,
               ITable& it, OTable& ot, const type_tree& tt, int pos = 0,
               const std::vector<int>& ignore_col_nums = empty_int_vec);
/**
 * like above but return an object Table.
 */
Table loadTable(const std::string& file_name, int pos = 0,
                const std::vector<int>& ignore_col_nums = empty_int_vec);

//////////////////
// ostreamTable //
//////////////////

// output the header of a data table in CSV format. target_pos is the
// column index of the target. If -1 then it is the last one.
std::ostream& ostreamTableHeader(std::ostream& out,
                                 const ITable& it, const OTable& ot,
                                 int target_pos = 0);

// output a data table in CSV format. Boolean values are output in
// binary form (0 for false, 1 for true). target_pos is the column
// index of the target. If -1 then it is the last one.
std::ostream& ostreamTable(std::ostream& out,
                           const ITable& it, const OTable& ot,
                           int target_pos = 0);
// like above but take a table instead of an input and output table
std::ostream& ostreamTable(std::ostream& out, const Table& table,
                           int target_pos = 0);

// like above but takes the file name where to write the table
void saveTable(const std::string& file_name,
               const ITable& it, const OTable& ot,
               int target_pos = 0);
// like above but take a table instead of a input and output table
void saveTable(const std::string& file_name, const Table& table,
               int target_pos = 0);

// like ostreamTableHeader but on a compressed table
std::ostream& ostreamCTableHeader(std::ostream& out, const CTable& ct);

// output a compress table in pseudo CSV format
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

/**
 * if the DSV data file has a header with labels
 */
std::vector<std::string> readInputLabels(const std::string& file, int pos = 0,
                                         const std::vector<int>& ignore_features
                                         = empty_int_vec);

std::ifstream* open_data_file(const std::string& fileName);

std::ostream& operator<<(std::ostream& out, const ITable& it);

std::ostream& operator<<(std::ostream& out, const OTable& ot);

std::ostream& operator<<(std::ostream& out, const complete_truth_table& tt);

std::ostream& operator<<(std::ostream& out, const CTable& ct);

}} // ~namespaces combo opencog

#endif // _OPENCOG_TABLE_IO_H
