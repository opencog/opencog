/*
 * load-file.cc
 *
 * Utility helper function -- load scheme code from a file
 * Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
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

#ifdef HAVE_GUILE

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <boost/filesystem/operations.hpp>

#include <opencog/guile/SchemeEval.h>
#include <opencog/util/Config.h>
#include <opencog/util/files.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>

namespace opencog {

/**
 * Load scheme code from a file.
 * The code will be loaded into a running instance of the evaluator.
 * Parsing errors will be printed to stderr.
 *
 * Return errno if file cannot be opened.
 */
int load_scm_file (AtomSpace& as, const std::string& filename)
{
#define BUFSZ 4120
	char buff[BUFSZ];

	FILE * fh = fopen (filename.c_str(), "r");
	if (NULL == fh)
	{
		int norr = errno;
		fprintf(stderr, "Error: %d %s: %s\n",
                norr, strerror(norr), filename.c_str());
		return norr;
	}

	SchemeEval* evaluator = new SchemeEval(&as);
	int lineno = 0;
	int pending_lineno = 0;

	while (1)
	{
		char * rc = fgets(buff, BUFSZ, fh);
		if (NULL == rc) break;
		std::string rv = evaluator->eval(buff);

		if (evaluator->eval_error())
		{
			fprintf(stderr, "File: %s line: %d\n", filename.c_str(), lineno);
			fprintf(stderr, "%s\n", rv.c_str());
			delete evaluator;
			return 1;
		}
		lineno ++;
		// Keep a record of where pending input starts
		if (!evaluator->input_pending()) pending_lineno = lineno;
	}

	if (evaluator->input_pending())
	{
		// Pending input... print error
		fprintf(stderr, "Warning file %s ended with unterminated "
		        "input begun at line %d\n", filename.c_str(), pending_lineno);
		delete evaluator;
		return 1;
	}

	fclose(fh);
	delete evaluator;
	return 0;
}

/**
 * Load scheme file, with the filename specified as a relative path,
 * and the search paths prepended to the relative path.  If the search
 * paths are null, a list of defaults search paths are used.
 */
int load_scm_file_relative (AtomSpace& as, const std::string& filename,
                            std::vector<std::string> search_paths)
{
    if (search_paths.empty())
        search_paths = DEFAULT_MODULE_PATHS;

    int rc = 2;
    for (const std::string& search_path : search_paths) {
        boost::filesystem::path modulePath(search_path);
        modulePath /= filename;
        logger().debug("Searching path %s", modulePath.string().c_str());
        if (boost::filesystem::exists(modulePath)) {
            rc = load_scm_file(as, modulePath.string());
            if (0 == rc) {
                logger().info("Loaded %s", modulePath.string().c_str());
                break;
            }
        }
    }

    if (rc)
    {
       logger().warn("Failed to load file %s: %d %s",
                     filename.c_str(), rc, strerror(rc));
    }
    return rc;
}

/**
 * Pull the names of scm files out of the config file, the SCM_PRELOAD
 * key, and try to load those, relative to the search paths.
 */
void load_scm_files_from_config(AtomSpace& atomSpace,
                                std::vector<std::string> search_paths)
{
    // Load scheme modules specified in the config file
    std::vector<std::string> scm_modules;
    tokenize(config()["SCM_PRELOAD"], std::back_inserter(scm_modules), ", ");

    std::vector<std::string>::const_iterator it;
    for (const std::string& scm_module : scm_modules)
        load_scm_file_relative(atomSpace, scm_module, search_paths);
}

}
#endif /* HAVE_GUILE */
