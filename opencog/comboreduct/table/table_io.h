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
 * Take a row, return a tokenizer.  Tokenization uses the
 * seperator characters comma, blank, tab (',', ' ' or '\t').
 */
typedef boost::tokenizer<boost::escaped_list_separator<char>> table_tokenizer;
table_tokenizer get_row_tokenizer(const std::string& line);

static const std::vector<unsigned> empty_unsigned_vec =
    std::vector<unsigned>();
static const std::vector<std::string> empty_string_vec =
    std::vector<std::string>();

/**
 * Visitor to parse a list of strings (buried in a multi_type_seq)
 * into a multi_type_seq containing the typed values given the input
 * type signature.
 */
builtin token_to_boolean(const std::string& token);
contin_t token_to_contin(const std::string& token);
vertex token_to_vertex(const type_node &tipe, const std::string& token);
struct from_tokens_visitor : public boost::static_visitor<multi_type_seq> {
    from_tokens_visitor(const std::vector<type_node>& types) : _types(types) {
        all_boolean = boost::count(types, id::boolean_type) == (int)types.size();
        all_contin = boost::count(types, id::contin_type) == (int)types.size();
    }
    result_type operator()(const string_seq& seq) {
        result_type res;
        if (all_boolean) {
            res = builtin_seq();
            builtin_seq& bs = res.get_seq<builtin>();
            boost::transform(seq, back_inserter(bs), token_to_boolean);
        }
        else if (all_contin) {
            res = contin_seq();
            contin_seq& cs = res.get_seq<contin_t>();
            boost::transform(seq, back_inserter(cs), token_to_contin);
        }
        else {
            res = vertex_seq();
            vertex_seq& vs = res.get_seq<vertex>();
            boost::transform(_types, seq, back_inserter(vs), token_to_vertex);
        }
        return res;
    }
    template<typename Seq> result_type operator()(const Seq& seq) {
        OC_ASSERT(false, "You are not supposed to do that");
        return result_type();
    }
    const std::vector<type_node>& _types;
    bool all_boolean, all_contin;
};

/**
 * parse a pair of key/value in a parse dataset, using ':' as
 * delimiter. For instance
 * 
 * parse_key_val("key : val")
 *
 * returns
 *
 * {"key", "val"}
 *
 * If no such delimiter is found then it return a pair with empty key
 * and empty val.
 */
std::pair<std::string, std::string> parse_key_val(std::string chunk);

/**
 * The class below tokenizes one row, and jams it into the table
 */
struct from_sparse_tokens_visitor : public from_tokens_visitor {
    from_sparse_tokens_visitor(const std::vector<type_node>& types,
                               const std::map<const std::string, size_t>& index,
                               size_t fixed_arity)
        : from_tokens_visitor(types), _index(index), _fixed_arity(fixed_arity) {}
    result_type operator()(const string_seq& seq) {
        using std::transform;
        using std::for_each;
        result_type res;
        if (all_boolean) {
            res = builtin_seq(_types.size(), id::logical_false);
            builtin_seq& bs = res.get_seq<builtin>();
            auto begin_sparse = seq.begin() + _fixed_arity;
            transform(seq.begin(), begin_sparse, bs.begin(), token_to_boolean);
            for (auto it = begin_sparse; it != seq.end(); ++it) {
                auto key_val = parse_key_val(*it);
                if (key_val != std::pair<std::string, std::string>()) {
                    size_t idx = _index.at(key_val.first);
                    bs[idx] = token_to_boolean(key_val.second);
                }
            }
        }
        else if (all_contin) {
            res = contin_seq(_types.size(), 0.0);
            contin_seq& cs = res.get_seq<contin_t>();
            auto begin_sparse = seq.cbegin() + _fixed_arity;
            transform(seq.begin(), begin_sparse, cs.begin(), token_to_contin);
            for (auto it = begin_sparse; it != seq.end(); ++it) {
                auto key_val = parse_key_val(*it);
                if (key_val != std::pair<std::string, std::string>()) {
                    size_t idx = _index.at(key_val.first);
                    cs[idx] = token_to_contin(key_val.second);
                }
            }
        }
        else {
            res = vertex_seq(_types.size());
            vertex_seq& vs = res.get_seq<vertex>();
            auto begin_sparse_types = _types.cbegin() + _fixed_arity;
            auto begin_sparse_seq = seq.cbegin() + _fixed_arity;
            transform(_types.begin(), begin_sparse_types,
                      seq.begin(), vs.begin(), token_to_vertex);
            for (auto it = begin_sparse_seq; it != seq.end(); ++it) {
                auto key_val = parse_key_val(*it);
                if (key_val != std::pair<std::string, std::string>()) {
                    size_t idx = _index.at(key_val.first);
                    vs[idx] = token_to_vertex(_types[idx], key_val.second);
                }
            }
        }
        return res;
    }
    std::map<const std::string, size_t> _index;
    size_t _fixed_arity;
};

/**
 * Take a line and return a vector containing the elements parsed.
 * Used by istreamTable.
 */
template<typename T>
std::vector<T> tokenizeRow(const std::string& line,
                           const std::vector<unsigned>& ignored_indices =
                           empty_unsigned_vec)
{
    table_tokenizer tok = get_row_tokenizer(line);
    std::vector<T> res;
    unsigned i = 0;
    for (const std::string& t : tok)
        if (!boost::binary_search(ignored_indices, i++))
            res.push_back(boost::lexical_cast<T>(t));
    return res;
}

/**
 * Take a line and return a pair with vector containing the input
 * elements and then output element.
 */
template<typename T>
std::pair<std::vector<T>, T> tokenizeRowIO(const std::string& line,
                                           const std::vector<unsigned>& ignored_indices = empty_unsigned_vec,
                                           unsigned target_idx = 0)
{
    std::pair<std::vector<T>, T> res;
    table_tokenizer tok = get_row_tokenizer(line);
    unsigned i = 0;
    for (const std::string& t : tok) {
        if (!boost::binary_search(ignored_indices, i)) {
            T el = boost::lexical_cast<T>(t);
            if (target_idx == i)
                res.second = el;
            else
                res.first.push_back(el);
        }
        i++;
    }
    return res;
}

//////////////////
// istreamTable //
//////////////////

// some hacky function to get the header of a DSV file (assuming there is one)
std::vector<std::string> get_header(const std::string& input_file);
        
std::istream& istreamRawITable(std::istream& in, ITable& tab,
                               const std::vector<unsigned>& ignored_indices =
                               empty_unsigned_vec)
    throw(std::exception, AssertionException);

std::istream& istreamITable(std::istream& in, ITable& tab,
                           const std::vector<std::string>& ignore_features);

std::istream& istreamTable(std::istream& in, Table& tab,
                           const std::string& target_feature,
                           const std::vector<std::string>& ignore_features);

// WARNING: this implementation only supports boolean ctable!!!!
std::istream& istreamCTable(std::istream& in, CTable& ctable);

/**
 * Load a OTable givent the file name. Only works for dense DSV data.
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
                const std::vector<std::string>& ignore_features
                = empty_string_vec);

type_node infer_type_from_token2(type_node curr_guess, const std::string& token);

std::istream& istreamDenseTable(std::istream& in, Table& tab,
                                const std::string& target_feature,
                                const std::vector<std::string>& ignore_features,
                                const type_tree& tt, bool has_header);

// WARNING: this implementation only supports boolean ctable!!!!
CTable loadCTable(const std::string& file_name);

//////////////////
// ostreamTable //
//////////////////

/// output the header of a data table in CSV format.
template<typename Out>
Out& ostreamTableHeader(Out& out, const ITable& it, const OTable& ot,
                        int target_pos)
{
    std::vector<std::string> header = it.get_labels();
    const std::string& ol = ot.get_label();
    OC_ASSERT(target_pos <= (int)header.size(),
              "target_pos %d greater than number of inputs %u",
              target_pos, header.size());
    if (target_pos < 0)
        header.push_back(ol);
    else
        header.insert(header.begin() + target_pos, ol);
    ostreamContainer(out, header, ",") << std::endl;
    return out;
}

/// output a data table in CSV format. Boolean values are output in
/// binary form (0 for false, 1 for true).
template<typename Out>
Out& ostreamTable(Out& out,
                  const ITable& it, const OTable& ot,
                  int target_pos = 0)
{
    // print header
    ostreamTableHeader(out, it, ot, target_pos);
    // print data
    OC_ASSERT(it.empty() || it.size() == ot.size());
    for (size_t row = 0; row < ot.size(); ++row) {
        std::vector<std::string> content;
        if (!it.empty())
            content = it[row].to_strings();
        std::string oc = table_fmt_vertex_to_str(ot[row]);
        if (target_pos < 0)
            content.push_back(oc);
        else
            content.insert(content.begin() + target_pos, oc);
        ostreamContainer(out, content, ",") << std::endl;
    }
    return out;
}

/// like above but take a table instead of an input and output table
template<typename Out>
Out& ostreamTable(Out& out, const Table& table)
{
    return ostreamTable(out, table.itable, table.otable, table.target_pos);
}

/// like above but take a table instead of a input and output table
void saveTable(const std::string& file_name, const Table& table);

/// output a compressed table in pseudo CSV format
std::ostream& ostreamCTableRow(std::ostream& out, const CTable::value_type& ctv);
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

std::ostream& operator<<(std::ostream& out, const Table& table);

std::ostream& operator<<(std::ostream& out, const CTable& ct);

std::ostream& operator<<(std::ostream& out, const complete_truth_table& tt);

}} // ~namespaces combo opencog

#endif // _OPENCOG_TABLE_IO_H
