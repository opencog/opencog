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

#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/count.hpp>
#include <boost/range/algorithm/binary_search.hpp>
#include <boost/range/algorithm_ext/for_each.hpp>
#include <boost/tokenizer.hpp>

#include "table.h"
#include "../type_checker/type_tree.h"

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
 * Convert strings to typed values
 */
builtin token_to_boolean(const std::string& token);
contin_t token_to_contin(const std::string& token);
vertex token_to_vertex(const type_node &tipe, const std::string& token);


// ===========================================================

static const std::vector<unsigned> empty_unsigned_vec =
    std::vector<unsigned>();
static const std::vector<std::string> empty_string_vec =
    std::vector<std::string>();


typedef boost::tokenizer<boost::escaped_list_separator<char>> table_tokenizer;

/**
 * Take a row, return a tokenizer.  Tokenization uses the
 * separator characters comma, blank, tab (',', ' ' or '\t').
 */
table_tokenizer get_row_tokenizer(const std::string& line);

/**
 * Take a line and return a vector containing the elements parsed.
 * Used by istreamTable.
 */
template<typename T>
static std::vector<T> tokenizeRow(const std::string& line,
                           const std::vector<unsigned>& ignored_indices =
                           empty_unsigned_vec)
{
    table_tokenizer tok = get_row_tokenizer(line);
    std::vector<T> res;
    unsigned i = 0;
    for (const std::string& t : tok) {

        // trim away whitespace padding; failing to do this
        // confuses stuff downstream.
        std::string clean(t);
        boost::trim(clean);

        // Sometimes the tokenizer returns pure whitespace :-(
        if (0 == clean.size()) continue;

        if (!boost::binary_search(ignored_indices, i++))
            res.push_back(boost::lexical_cast<T>(clean));
    }
    return res;
}

// ===========================================================

//////////////////
// istreamTable //
//////////////////

// some hacky function to get the header of a DSV file (assuming there is one)
std::vector<std::string> get_header(const std::string& input_file);
        
std::istream& istreamRawITable(std::istream& in, ITable& tab,
                               const std::vector<unsigned>& ignored_indices =
                               empty_unsigned_vec)
    throw(std::exception);

std::istream& istreamITable(std::istream& in, ITable& tab,
                           const std::vector<std::string>& ignore_features);

std::istream& istreamTable(std::istream& in, Table& tab,
                           const std::string& target_feature,
                           const std::string& timestamp_feature,
                           const std::vector<std::string>& ignore_features);

// WARNING: this implementation only supports boolean ctable!!!!
std::istream& istreamCTable(std::istream& in, CTable& ctable);

/**
 * Load a OTable given the file name. Only works for dense DSV data.
 */
OTable loadOTable(const std::string& file_name,
                  const std::string& target_feature);

// TODO: reimplement loadITable with the same model of loadTable and
// remove loadITable_optimized
ITable loadITable(const std::string& file_name,
                  const std::vector<std::string>& ignore_features
                  = empty_string_vec);

ITable loadITable_optimized(const std::string& file_name,
                            const std::vector<std::string>& ignore_features
                            = empty_string_vec);

/**
 * If target_feature is empty then, in case there is no header, it is
 * assumed to be the first feature.
 */
Table loadTable(const std::string& file_name,
                const std::string& target_feature = std::string(),
                const std::string& timestamp_feature = std::string(),
                const std::vector<std::string>& ignore_features
                = empty_string_vec);

std::istream& istreamDenseTable(std::istream& in, Table& tab,
                                const std::string& target_feature,
                                const std::string& timestamp_feature,
                                const std::vector<std::string>& ignore_features,
                                const type_tree& tt, bool has_header);

// WARNING: this implementation only supports boolean ctable!!!!
CTable loadCTable(const std::string& file_name);

//////////////////
// ostreamTable //
//////////////////

/// output the header of a data table in CSV format.
template<typename Out>
Out& ostreamTableHeader(Out& out, const Table& table)
{
    // Add input features in header
    std::vector<std::string> header = table.itable.get_labels();
    unsigned hsize = header.size();

    // Add target feature in header
    const std::string& ol = table.otable.get_label();
    header.insert(header.begin() + std::min(table.target_pos, hsize), ol);

    // Add timestamp feature in header
    if (!table.ttable.empty()) {
        const std::string& tl = table.ttable.get_label();
        header.insert(header.begin() + table.timestamp_pos, tl);
    }

    // Write the header
    ostreamContainer(out, header, ",") << std::endl;
    return out;
}

/// Output a data table in CSV format. Boolean values are output in
/// binary form (0 for false, 1 for true).
template<typename Out>
Out& ostreamTable(Out& out, const Table& table)
{
    // print header
    ostreamTableHeader(out, table);

    // print data
    unsigned isize = table.itable.size(), osize = table.otable.size();
    OC_ASSERT(table.itable.empty() || isize == osize);
    for (size_t row = 0; row < osize; ++row) {
        // Add input values
        std::vector<std::string> content;
        if (!table.itable.empty())
            content = table.itable[row].to_strings();
        unsigned csize = content.size();

        // Add target feature value
        std::string oc = table_fmt_vertex_to_str(table.otable[row]);
        content.insert(content.begin() + std::min(table.target_pos, csize), oc);

        // Add timestamp feature value
        if (!table.ttable.empty()) {
            std::string tc = TTable::to_string(table.ttable[row]);
            content.insert(content.begin() + table.timestamp_pos, tc);
        }

        // Write content row
        ostreamContainer(out, content, ",") << std::endl;
    }
    return out;
}

/// like above but take a table instead of a input and output table
void saveTable(const std::string& file_name, const Table& table);

/// output a compressed table in pseudo CSV format
std::ostream& ostreamCTableRow(std::ostream& out, const CTable::value_type& ctv);
std::ostream& ostreamCTable(std::ostream& out, const CTable& ct);

/// Output a compressed table with each row corresponding to a
/// timestamp, chronologically ordered.
std::ostream& ostreamCTableTime(std::ostream& out, const CTableTime& ctt);

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

std::ostream& operator<<(std::ostream& out, const Table& table);

std::ostream& operator<<(std::ostream& out, const CTable& ct);

std::ostream& operator<<(std::ostream& out, const complete_truth_table& tt);

}} // ~namespaces combo opencog

#endif // _OPENCOG_TABLE_IO_H
