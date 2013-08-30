/*
 * src/AtomSpace/utils.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#ifndef OPENCOG_FILE_LIST_H
#define OPENCOG_FILE_LIST_H

#include <vector>

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

//! a list of files
/**
 * Each entry in the list contains:
 * - the path that was provided in constructor or to the method that created the entry 
 * - a slash (/)
 * - the name of that particular entry.
 *
 * So:
 * @code
 * FileList("/foo");
 * @endcode
 * may have an entry: /foo/bar, and
 * @code
 * FileList("../../foo");
 * @endcode
 * may have an entry: ../../foo/bar
 *
 */
class FileList
{

private:

    //! the list
    std::vector<char *>fileList;

public:

    //! constructor; loads the file(s) in the list
    /**
     * If the input is a file, that file alone is pushed into the list; if
     * the input is a directory, then all files and directories will be added;
     * unlike getAllFilesRecursively(), this sets only direct kids of the directory.
     */
    FileList(const char* path) throw (IOException);
    //! constructor; creates an empty list
    FileList();
    //! destructor
    ~FileList();

    //! create a file list structure with all files and directories
    /**
     * \warning The ownership of the returned pointer goes to the caller.
     *
     * @todo trivial: create a helper method to avoid new-delete on each step
     */
    static FileList *getAllFilesRecursively(const char* );
    
    //! number of entries in the list
    unsigned int getSize();
    
    //! file at a specific index
    const char* getFile(unsigned int) throw (IndexErrorException);
};

/** @}*/
} // namespace opencog

#endif /* OPENCOG_FILE_LIST_H */
