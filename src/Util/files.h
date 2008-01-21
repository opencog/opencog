#ifndef _PB_FILES_H_
#define _PB_FILES_H_
/**
 * Files.h
 *
 * Functions for manipulating files
 * 
 */

/**
 * Check if a file exists in PetDatabase (currently a directory)
 * 
 * @param filemane The name of the file to check if it exists.
 * @return true if file exists, false otherwise
 */        
bool fileExists(const char* filename);

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

#endif //_PB_FILES_H_
