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

#include "platform.h"
#include "utils.h"

#include <stdlib.h>
#include <stdarg.h>
#include <string>
#include <time.h>
#ifndef WIN32
#include <sys/time.h>
#endif

using namespace opencog;

std::string opencog::toString(double data)
{
    char buffer[256];
    sprintf(buffer, "%f", data);
    return buffer;
}

std::string opencog::bold(int i)
{
    char buf[256];
    sprintf(buf, "[1m%d[0m", i);
    return buf;
}

std::string opencog::bold(const char* str)
{
    char buf[512];
    sprintf(buf, "[1m%s[0m", str);
    return buf;
}

std::string opencog::padstr(const char* s, unsigned int size, padAlignment a) throw (InvalidParamException)
{
    std::string answer;
    switch (a) {
    case CENTER: {
        unsigned int lpadding = (size - strlen(s)) / 2;
        unsigned int rpadding = lpadding;
        if (lpadding + rpadding + strlen(s) != size) rpadding++;
        char* charBuf = new char[size + 1];
        for (unsigned int i = 0; i < size; i++) charBuf[i] = ' ';
        for (unsigned int i = 0; i < strlen(s); i++) charBuf[i + lpadding] = s[i];
        charBuf[size] = '\0';
        answer = charBuf;
        delete[](charBuf);
        return answer;
    }
    case LEFT: {
        unsigned int rpadding = size - strlen(s);
        char* charBuf = new char[size + 1];
        for (unsigned int i = rpadding; i < size; i++) charBuf[i] = ' ';
        for (unsigned int i = 0; i < strlen(s); i++) charBuf[i] = s[i];
        charBuf[size] = '\0';
        answer = charBuf;
        delete[](charBuf);
        return answer;
    }
    case RIGHT: {
        unsigned int lpadding = size - strlen(s);
        char* charBuf = new char[size + 1];
        for (unsigned int i = 0; i < lpadding - 1; i++) charBuf[i] = ' ';
        for (unsigned int i = 0; i < strlen(s); i++) charBuf[i + lpadding] = s[i];
        charBuf[size] = '\0';
        answer = charBuf;
        delete[](charBuf);
        return answer;
    }
    default:
        throw InvalidParamException(TRACE_INFO, "utils - Invalid padAlignmet parameter.");
        break;
    }
    return (answer);
}

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

#endif // !WIN32

const char* opencog::nextLine(const char *from, std::string &line)
{
    line = "";
    if (from != NULL) {
        // Gets the first line
        const char* lineStart = from;
        int lineLength = 0;
        while ((*from != '\n') && ((*from != '\0'))) {
            from++; lineLength++;
        }
        line = std::string(lineStart, lineLength);
        // Points return value to the next line
        if (*from == '\0') {
            from = NULL;
        } else {
            from++;
        }
    }
    return(from);
}

// TODO: Review this method for both 32/64-bit processor compatibility
int opencog::bitcount(unsigned long n)
{
    /* works for 32-bit numbers only    */
    /* fix last line for 64-bit numbers */

    register unsigned long tmp;

    tmp = n - ((n >> 1) & 033333333333)
          - ((n >> 2) & 011111111111);
    return ((tmp + (tmp >> 3)) & 030707070707) % 63;
}

/** Additions by Ari (March 20) */

#include "utils2.h"

#ifdef WIN32

tree<Vertex> MakeVirtualAtom_slow(Type T, tree<Vertex> t1, tree<Vertex> t2, tree<Vertex> t3, tree<Vertex> t4)
{
    try {
        tree<Vertex> ret;
        ret.set_head(Vertex((Handle)T));
        ret.append_child(ret.begin(), t1.begin());
        ret.append_child(ret.begin(), t2.begin());
        ret.append_child(ret.begin(), t3.begin());
        ret.append_child(ret.begin(), t4.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
}
tree<Vertex> MakeVirtualAtom_slow(Type T, tree<Vertex> t1, tree<Vertex> t2, tree<Vertex> t3)
{
    try {
        tree<Vertex> ret;
        ret.set_head(Vertex((Handle)T));
        ret.append_child(ret.begin(), t1.begin());
        ret.append_child(ret.begin(), t2.begin());
        ret.append_child(ret.begin(), t3.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
}

tree<Vertex> MakeVirtualAtom_slow(Type T, tree<Vertex> t1, tree<Vertex> t2)
{
    try {
        tree<Vertex> ret;
        ret.set_head(Vertex((Handle)T));
        ret.append_child(ret.begin(), t1.begin());
        ret.append_child(ret.begin(), t2.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
}

tree<Vertex> MakeVirtualAtom_slow(Type T, tree<Vertex> t1)
{
    try {
        tree<Vertex> ret;
        ret.set_head(Vertex((Handle)T));
        ret.append_child(ret.begin(), t1.begin());

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
}

tree<Vertex> MakeVirtualAtom_slow(Type T)
{
    try {
        tree<Vertex> ret;
        ret.set_head(Vertex((Handle)T));

        return ret;

    } catch (...) {
        puts("MakeVirtualAtom_slow exception."); getc(stdin);
    }
}
#endif


bool less_tree_vertex::operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const
{
    return (*this)(lhs, rhs, lhs.begin(), rhs.begin());
}
int ddd = 0;
bool less_tree_vertex::operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs,
                                  tree<Vertex>::iterator ltop,
                                  tree<Vertex>::iterator rtop) const
{
    /*  if (ddd)
    {
    raw_print(*const_cast<tree<Vertex>*>(&lhs), const_cast<tree<Vertex>*>(&lhs)->begin(), 0);
    raw_print(*const_cast<tree<Vertex>*>(&rhs), const_cast<tree<Vertex>*>(&rhs)->begin(), 0);
    }*/

    if (*ltop < *rtop)
        return true;
    if (*rtop < *ltop)
        return false;

    if (ltop.number_of_children() < rtop.number_of_children())
        return true;
    if (ltop.number_of_children() > rtop.number_of_children())
        return false;

    tree<Vertex>::sibling_iterator r_i = rhs.begin(rtop);
    for (tree<Vertex>::sibling_iterator i = lhs.begin(ltop);
            i != lhs.end(ltop);
            i++) {
        if (less_tree_vertex()(lhs, rhs, i, r_i))
            return true;
        else
            if (less_tree_vertex()(rhs, lhs, r_i, i))
                return false;

        r_i++;
    }

    return false;
}

bool LoadTextFile(const string fname, string& dest)
{
    FILE *f = fopen(fname.c_str(), "rt");
    if (f == NULL) {
        puts("File not found.");
        return false;
    }
    fseek(f, 0L, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0L, SEEK_SET);

    char *buf = new char[fsize+2];
    long bptr = 0;

    while (!feof(f))
        buf[bptr++] = getc(f);
// fread(buf, 8000, 1+(fsize/8000), f);
    buf[bptr] = '\0';

    fclose(f);

    dest = buf;

    delete[] buf;

    return true;
}

string toupper(string k)
{
    const char* data = k.c_str();
    char buf[1000];

    //iterator<string,b> iii;

    unsigned int i;
    for (i = 0; i < k.size(); i++)
        buf[i] = Isox(data[i]);
    buf[i] = 0;

    return buf;
}

std::string XMLembed(const std::string &elem, const std::string &pcdata)
{
    return std::string("<")
           + elem
           + std::string("> ")
           + std::string(pcdata)
           + std::string(" </")
           + std::string(elem)
           + std::string(">");
};

/* MISC UTILITIES */

bool visible(char c)
{
    return c != ' ' && c != '\r' && c != '\n' && c != '\t' && c != 0;
}

string i2str(int d)
{
    char temp[20];
    sprintf(temp, "%d", d);
    return temp;
}

bool exists(const char *fname)
{
    FILE* f = fopen(fname, "rb");
    if (!f)
        return false;
    fclose(f);
    return true;
}

char opencog::Isox(char s)
{
    if (s > 96 && s < 123) s -= 32;
    return s;
}

nocase_string::nocase_string(const char* str) : string(str) { }
nocase_string::nocase_string(const nocase_string& str) : string(str.c_str()) { }
nocase_string::nocase_string(const string& str) : string(str) { }
nocase_string::nocase_string() { }

bool nocase_string::operator <(const char* rhs)
{

    if (nocase_equal(this->c_str(), rhs)) {
        return false; //equal
    } else {
        return strcmp(this->c_str(), rhs);
    }
}

bool nocase_string::operator <(const nocase_string& rhs)
{
    return ((*this) < rhs.c_str());
}

bool nocase_string::operator <(const string& rhs)
{
    return ((*this) < rhs.c_str());
}

bool nocase_string::operator ==(const char* rhs)
{
    return nocase_equal(this->c_str(), rhs);
}

bool nocase_string::operator ==(const nocase_string& rhs)
{
    return nocase_equal(this->c_str(), rhs.c_str());
}

bool nocase_string::operator ==(const string& rhs)
{
    return nocase_equal(this->c_str(), rhs.c_str());
}

bool nocase_string::operator !=(const char* rhs)
{
    return !(*this == rhs);
}
bool nocase_string::operator !=(const nocase_string& rhs)
{
    return !(*this == rhs);
}
bool nocase_string::operator !=(const string& rhs)
{
    return !(*this == rhs);
}
void nocase_string::operator +=(nocase_string s)
{
    string::operator+=(s);
}
nocase_string nocase_string::operator+(nocase_string s)
{
    return nocase_string(string(*this) + string(s));
}

bool opencog::nocase_equal(const char *s1, const char *s2)
{
    int i;
    for (i = 0; s1[i] != 0 && s2[i] != 0;i++)
        if (Isox(s1[i]) != Isox(s2[i]))
            return false;

    return (s1[i] == 0 && s2[i] == 0);
}

StringTokenizer::StringTokenizer(const string &rStr, const string &rDelimiters)
{
    string::size_type lastPos(rStr.find_first_not_of(rDelimiters, 0));
    string::size_type pos(rStr.find_first_of(rDelimiters, lastPos));
    while (string::npos != pos || string::npos != lastPos) {
        push_back(rStr.substr(lastPos, pos - lastPos));
        lastPos = rStr.find_first_not_of(rDelimiters, pos);
        pos = rStr.find_first_of(rDelimiters, lastPos);
    }
}

std::vector<string> StringTokenizer::WithoutEmpty() const
{
    std::vector<string> ret;

    for (unsigned int i = 0; i < this->size(); i++)
        if (!(*this)[i].empty())
            ret.push_back((*this)[i]);

    return ret;
}


/**
 * Time used as reference to set/get timestamps over the code
 */
static timeval referenceTime;
static bool referenceTimeInitialized = false;

void initReferenceTime()
{
    gettimeofday(&referenceTime, NULL);
    referenceTimeInitialized = true;
}

unsigned long getElapsedMillis()
{
    cassert(TRACE_INFO, referenceTimeInitialized,
            "utils - refenceTimeInitialized should have been initialized.");
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return (currentTime.tv_sec -referenceTime.tv_sec)*1000 + (currentTime.tv_usec - referenceTime.tv_usec) / 1000;
}

bool less_vtree_it( const tree<Vertex> & lhs_t, const tree<Vertex> & rhs_t,
                    tree<Vertex>::sibling_iterator ltop, tree<Vertex>::sibling_iterator rtop)
{
    if ((*ltop) < (*rtop))
        return true;
    if ((*rtop) < (*ltop))
        return false;

    tree<Vertex>::sibling_iterator rit = rhs_t.begin(rtop);

    for (tree<Vertex>::sibling_iterator lit = lhs_t.begin(ltop);
            lit != lhs_t.end  (ltop);
            lit++, rit++)
        if (less_vtree_it(lhs_t, rhs_t, lit, rit))
            return true;
        else if (less_vtree_it(lhs_t, rhs_t, rit, lit))
            return false;

    return false;
}

bool less_vtree::operator()(const tree<Vertex>& lhs, const tree<Vertex>& rhs) const
{
    if (lhs.size() < rhs.size())
        return true;
    if (lhs.size() > rhs.size() || lhs.empty())
        return false;

    return less_vtree_it(lhs, rhs, lhs.begin(), rhs.begin());
}

