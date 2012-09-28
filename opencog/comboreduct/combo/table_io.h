/**
 * table_io.h ---
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


#ifndef _OPENCOG_TABLE_IO_H
#define _OPENCOG_TABLE_IO_H

#include <fstream>
#include <string>
#include <vector>

#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/binary_search.hpp>
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
 * Take a row, return a tokenizer.  Tokenization uses the
 * seperator characters comma, blank, tab (',', ' ' or '\t').
 */
typedef boost::tokenizer<boost::escaped_list_separator<char>> table_tokenizer;
table_tokenizer get_row_tokenizer(const std::string& line);

/**
 * Take a line and return a vector containing the elements parsed.
 * Used by istreamTable. This will modify the line to remove leading
 * non-ASCII characters, as well as stripping of any carriage-returns.
 */
template<typename T>
std::vector<T> tokenizeRow(const std::string& line)
{
    table_tokenizer tok = get_row_tokenizer(line);
    std::vector<T> res;
    foreach (const std::string& t, tok)
        res.push_back(boost::lexical_cast<T>(t));
    return res;
}
// like tokenizerRow but take a vector of indices to ignore, this
// duplication is here possible temporary in case that method is
// substantially slower than tokenizerRow when ignored_indices is
// empty. As soon as it is measured it is not (which I think is the
// case because of branch prediction) then it can replace tokenizerRow
// (and rename to it) with an empty ignore_indices as default.
template<typename T>
std::vector<T> tokenizeRow_ignore_indices(const std::string& line,
                                          const std::vector<unsigned>& ignored_indices)
{
    table_tokenizer tok = get_row_tokenizer(line);
    std::vector<T> res;
    unsigned i = 0;
    foreach (const std::string& t, tok)
        if (!boost::binary_search(ignored_indices, i++))
            res.push_back(boost::lexical_cast<T>(t));
    return res;
}
        
//////////////////
// istreamTable //
//////////////////

// some hacky function to get the header of a DSV file (assuming there is one)
std::vector<std::string> get_header(const std::string& input_file);
        
/// Read a table from an input stream.
std::istream& istreamOTable(std::istream& in, OTable& tab,
                            const std::string& target_feature);
        
std::istream& istreamRawITable(std::istream& in, ITable& tab)
    throw(std::exception, AssertionException);

// like istreamRawITable but take a vector of indices to ignore, this
// duplication is here possible temporary in case that method is
// substantially slower than istreamRawITable when ignored_indices is
// empty. As soon as it is measured it is not (which I think is the
// case because of branch prediction) then it can replace
// istreamRawITable (and rename to it) with an empty ignore_indices as
// default.
std::istream& istreamRawITable_ingore_indices(std::istream& in, ITable& tab,
                                              const std::vector<unsigned>& ignored_indices)
    throw(std::exception, AssertionException);

std::istream& istreamITable(std::istream& in, ITable& tab,
                           const std::vector<std::string>& ignore_features);

std::istream& istreamTable(std::istream& in, Table& tab,
                           const std::string& target_feature,
                           const std::vector<std::string>& ignore_features);

/**
 * like above but read from file, and return an object Table.
 */
OTable loadOTable(const std::string& file_name,
                  const std::string& target_feature);
        
ITable loadITable(const std::string& file_name,
                  const std::vector<std::string>& ignore_features);

Table loadTable(const std::string& file_name,
                const std::string& target_feature,
                const std::vector<std::string>& ignore_features);


//////////////////
// ostreamTable //
//////////////////

/// output a data table in CSV format. Boolean values are output in
/// binary form (0 for false, 1 for true).
std::ostream& ostreamTable(std::ostream& out,
                           const ITable& it, const OTable& ot,
                           int target_pos = 0);

/// like above but take a table instead of an input and output table
std::ostream& ostreamTable(std::ostream& out, const Table& table,
                           int target_pos = 0);

/// like above but take a table instead of a input and output table
void saveTable(const std::string& file_name, const Table& table,
               int target_pos = 0);

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

std::ostream& operator<<(std::ostream& out, const ITable& it);

std::ostream& operator<<(std::ostream& out, const OTable& ot);

std::ostream& operator<<(std::ostream& out, const CTable& ct);

std::ostream& operator<<(std::ostream& out, const complete_truth_table& tt);

}} // ~namespaces combo opencog

#endif // _OPENCOG_TABLE_IO_H
