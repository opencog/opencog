/*
 * opencog/util/files.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#ifndef _OPENCOG_FILES_H_
#define _OPENCOG_FILES_H_

/**
 * \file files.h
 *
 * Functions for manipulating files
 *
 */

#include <string>

namespace opencog
{
/** \addtogroup grp_cogutil
 *  @{
 */

/**
 * Set of paths to search when attempting to load a module.
 * This is used both for loading shared libs and scheme files.
 */
extern const char** DEFAULT_MODULE_PATHS;

/**
 * Check if a file exists in the current directory
 *
 * @param filename The name of the file to check if it exists.
 * @return true if file exists, false otherwise
 */
bool fileExists(const char* filename);

/** Checks whether a file exists */
bool exists(const char *fname);


/**
 * Expand the path string replacing any ocurrency of the $USER flag with the
 * current system user information. If no user is found, unknown_user is place.
 *
 * @param path A std::string representing the path to be expanded.
 *
 * IMPORTANT:
 * This function modifies the content of the std::string passed as parameter.
 */
void expandPath(std::string& path);

/**
 * Creates the directory structure based on the path provided.
 *
 * @param directory The directory path
 * @return true if the directory is created or already exists. False otherwise
 */
bool createDirectory(const char* filename);

/**
 * Appends the content of a file with the given name to a string
 * @return true if the file was successfully read into the string
 */
bool appendFileContent(const char* filename, std::string &s);

/** Load the contents of a textfile \param fname to \param dest. */
bool LoadTextFile(const std::string fname, std::string& dest);

/**
 * Get the file name (including absolute path) of current executing file 
 */
std::string getExeName();

/**
 * Get the absolute path (including "/" at the end) where the current executing file runs
 */
std::string getExeDir(); 

/** @}*/
} // namespace opencog

#endif //_OPENCOG_FILES_H_
