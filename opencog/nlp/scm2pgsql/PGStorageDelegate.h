/*
 * PGStorageDelegate.h
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#ifndef _OPENCOG_PGSTORAGE_DELEGATE_H
#define _OPENCOG_PGSTORAGE_DELEGATE_H

//#include <opencog/persist/sql/postgres/PGAtomStorage.h>

#include <string>

namespace opencog
{

/**
 * A helper class that implements a method to load a HUGE .scm file and
 * store it in the passed DB using (pgsql-store).
 */
class PGStorageDelegate 
{

public:

    ~PGStorageDelegate();

    /**
     * This constructor expects the DB parameters required in (pgsql-open).
     *
     * @param dbName The name of the Postgres database 
     * @param userName Logname name in the DB server
     * @param password Of the given logname
     */
    PGStorageDelegate(const char *dbName, const char *userName, const char *password);

    /**
     * Load the atoms from the passed .scm file and store them in the
     * database passed in the constructor. This command will:
     *     (1) Create an empty AtomSpace
     *     (2) Use SCMLoad to populate this AtomSpace with the atoms from the
     *         passed file
     *     (3) Call (pgsql-open) (pgsql-store) and (pssql-close) once after the
     *     whole file is load.
     *
     * @return true if the passed file is somehow unusabel or false otherwise
     * @param fileName Full path of the SCM file to be loaded
     */
    bool loadSCMFile(const char *fileName);

private:

    std::string _dbName;
    std::string _userName;
    std::string _password;
};

}

#endif // _OPENCOG_PGSTORAGE_DELEGATE_H
