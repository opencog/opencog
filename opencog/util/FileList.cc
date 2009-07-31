/*
 * src/AtomSpace/utils.cc
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
#include "FileList.h"
#include <opencog/util/platform.h>

using namespace opencog;

#ifndef WIN32

#include <dirent.h>

FileList *FileList::getAllFilesRecursively(const char* arg)
{

    //printf("Getting files from <%s>\n", arg);
    FileList *answer = new FileList();
    DIR *d = opendir(arg);
    if (d == NULL) {
        //printf("is file\n");
        //is file
        answer->fileList.push_back(strdup(arg));
        return answer;
    } else {
        //printf("is dir\n");
        // is dir
        struct dirent *dp;
        while ((dp = readdir(d)) != NULL) {
            if (!strcmp(dp->d_name, ".") || (!strcmp(dp->d_name, ".."))) {
                continue;
            }
            char buf[1 << 16];
            sprintf(buf, "%s", arg);
            if (arg[strlen(arg) - 1] != '/') strcat(buf, "/");
            strcat(buf, dp->d_name);
            FileList *aux = getAllFilesRecursively(buf);
            for (unsigned int i = 0; i < aux->fileList.size(); i++) {
                answer->fileList.push_back(strdup(aux->fileList[i]));
            }
            delete aux;
        }
        closedir(d);
        return answer;
    }
}

FileList::FileList()
{
}

FileList::FileList(const char* arg) throw (IOException)
{

    DIR *d = opendir(arg);
    if (d != NULL) {
        struct dirent *dp;
        while ((dp = readdir(d)) != NULL) {
            if (!strcmp(dp->d_name, ".") || (!strcmp(dp->d_name, ".."))) {
                continue;
            }
            char buf[1 << 16];
            sprintf(buf, "%s", arg);
            if (arg[strlen(arg) - 1] != '/') strcat(buf, "/");
            strcat(buf, dp->d_name);
            fileList.push_back(strdup(buf));
        }
        closedir(d);
    } else {
        FILE *f = fopen(arg, "r");
        if (f == NULL) {
            throw IOException(TRACE_INFO, "utils - Unable to open file or directory '%s'.", arg);
        }
        fclose(f);
        fileList.push_back(strdup(arg));
    }
}

FileList::~FileList()
{
    for (unsigned int i = 0; i < fileList.size(); i++) {
        free(fileList[i]);
    }
}

unsigned int FileList::getSize()
{
    return fileList.size();
}

const char* FileList::getFile(unsigned int i) throw (IndexErrorException)
{
    if (i < 0 || i >= fileList.size()) {
        throw IndexErrorException(TRACE_INFO, "utils - File index out of bounds: '%d'", i);
    }
    return fileList[i];
}

#endif // not WIN32

