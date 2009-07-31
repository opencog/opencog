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

class FileList
{

private:

    std::vector<char *>fileList;

public:

    FileList(const char* ) throw (IOException);
    FileList();
    ~FileList();

    static FileList *getAllFilesRecursively(const char* );

    unsigned int getSize();
    const char* getFile(unsigned int) throw (IndexErrorException);
};

} // namespace opencog

#endif /* OPENCOG_FILE_LIST_H */
