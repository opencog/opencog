/**
 * problem.h ---
 *
 * Copyright (C) 2013,2014 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_MOSES_PROBLEM_H
#define _OPENCOG_MOSES_PROBLEM_H

#include <string>
#include <boost/program_options.hpp>
#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace moses {

class option_base
{
    public:
        virtual ~option_base() {}
        virtual void add_options(boost::program_options::options_description&) = 0;
        virtual void parse_options(boost::program_options::variables_map&) {};
};

class option_manager
{
	public:
		void register_options(option_base*);
		void init_options();
		void parse_options(int argc, char* argv[]);
	private:
		std::set<option_base*> _option_set;
		boost::program_options::options_description _desc;
};

class problem_base
{
    public:
        virtual ~problem_base() {}
        virtual const std::string name() const = 0;
        virtual const std::string description() const = 0;
        virtual void run(option_base*) = 0;
};

class problem_manager
{
    public:
        ~problem_manager();
        void register_problem(problem_base*);
        problem_base* find_problem(const std::string&);
    private:
        std::map<std::string, problem_base*> _problem_set;
};


// misc utility .. doesn't really below here,,, ?
unsigned alphabet_size(const combo::type_tree& tt,
                       const combo::vertex_set ignore_ops);


} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_PROBLEM_H
