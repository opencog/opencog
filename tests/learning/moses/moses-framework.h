/** moses-framework.h ---
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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

#include <cxxtest/TestSuite.h>
#include <opencog/learning/moses/main/moses_exec.h>
#include <boost/range/algorithm/find_if.hpp>

using namespace opencog::moses;

string build_arguments(vector<string>& arguments)
{
    char tempfile[] = "/tmp/mosesUTestXXXXXX";
    mkstemp(tempfile);
    arguments.insert(arguments.begin(), "moses-exec");
    arguments.push_back(string("-o").append(tempfile).c_str());
    return tempfile;
}

pair<score_t, combo_tree> parse_result(const string& tempfile)
{
    // open file
    FILE* fp = fopen(tempfile.c_str(), "r");
    __gnu_cxx::stdio_filebuf<char> pipe_buf(fp, ios_base::in);
    istream in(&pipe_buf);
    // parse result
    score_t score;
    in >> score;
    combo_tree tr;
    in >> tr;
    cout << score << " " << tr << endl;
    // close file and return result
    pclose(fp);
    return {score, tr};
}

void moses_test_score(vector<string> arguments, score_t expected_sc = 0)
{
    // build arguments
    string tempfile = build_arguments(arguments);
    // run moses
    moses_exec(arguments);
    // parse the result
    auto result = parse_result(tempfile);
    // check that the result is the expected one
    TS_ASSERT_LESS_THAN(fabs(result.first - expected_sc), 1.0e-8);
}

// test that the first candidate is one of the expected combo tree
void moses_test_combo(vector<string> arguments,
                      vector<string> expected_tr_strs)
{
    // build arguments
    string tempfile = build_arguments(arguments);
    // run moses
    moses_exec(arguments);
    // parse the result
    auto result = parse_result(tempfile);
    // check that the result is one of the expected ones
    auto f_it = boost::find_if(expected_tr_strs,
                               [&](const string& tr_str) {
                                   combo_tree tr;
                                   stringstream(tr_str) >> tr;
                                   return tr == result.second;});
    TS_ASSERT(f_it != expected_tr_strs.end());
}

