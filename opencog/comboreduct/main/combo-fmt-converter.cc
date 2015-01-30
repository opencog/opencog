/*
 * opencog/comboreduct/main/combo-fmt-converter.cc
 *
 * Copyright (C) 2015 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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

#include <boost/program_options.hpp>

#include "../combo/iostream_combo.h"
#include "../type_checker/type_tree.h"

using namespace boost::program_options;
using namespace std;
using namespace opencog;
using namespace combo;
// using namespace ant_combo;

// Structure containing the options for the combo-fmt-converter program
struct pgrParameters
{
	vector<string> combo_programs;
	vector<string> combo_programs_files;
	string output_file;
	string output_format_str;
};

/**
 * Read and parse the combo-fmt-converter program arguments.
 * Return the parsed results in the parameters struct.
 */
pgrParameters parse_program_args(int argc, char** argv)
{
	// program options, see options_description below for their meaning
	pgrParameters pa;

	// Declare the supported options.
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "Produce help message.\n")

		// (opt_desc_str(combo_str_opt).c_str(),
		//  value<vector<string>>(&pa.combo_programs),
		//  "Alternative way to entering the programs to convert (as opposed to "
		//  "via stdin). "
		//  "Remember to escape the $ signs or to wrap them in single "
		//  "quotes as to not let the shell perform variable expansions "
		//  "on the combo program variables."
		//  "This option may be "
		//  "used several times to convert several programs.\n")

		// (opt_desc_str(combo_prog_file_opt).c_str(),
		//  value<vector<string>>(&pa.combo_programs_files),
		//  "Other alternative way. Indicate the path of a file containing combo "
		//  "programs (seperated by newlines) to convert. "
		//  "This option may be used several times to progress several files.\n")

		// (opt_desc_str(output_file_opt).c_str(),
		//  value<string>(&pa.output_file),
		//  "File where to save the results. If empty then outputs on "
		//  "stdout.\n")

		("output-format,f",
		 value<string>(&pa.output_format_str)->default_value("combo"),
		 "Supported output formats are combo, python and scheme.\n")

		;

	variables_map vm;
	store(parse_command_line(argc, argv, desc), vm);
	notify(vm);

	if (vm.count("help") || argc == 1) {
		cout << desc << "\n";
		exit(1);
	}

	return pa;
}

int main(int argc, char** argv)
{
	pgrParameters pa = parse_program_args(argc, argv);

	// Parse output format
	combo::output_format fmt = parse_output_format(pa.output_format_str);

	// IO loop
	while (cin.good()) {
		// Parse combo tree
		string line;
		getline(cin, line);
		if (line.empty())
			continue;

		vector<string> variables = parse_combo_variables(line);
		combo_tree tr = str2combo_tree(line, variables);
		type_tree tt = infer_type_tree(tr);

		// Write to the right format
		ostream_combo_tree(cout, tr, variables, fmt) << std::endl;
	}
	
	return 0;	
}
