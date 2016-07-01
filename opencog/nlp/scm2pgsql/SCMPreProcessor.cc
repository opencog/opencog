/*
 * SCMPreProcessor.cc
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

#include <stdio.h>
#include <fstream>

void parseFile(std::fstream &fin)
{
    int level = 0;
    char c;
    std::string line = "";
    while (fin >> std::noskipws >> c) {
        if (c != '\n') {
            line += c;
        }
        switch (c) {
            case '(': {
                level++;
                break;
            }
            case ')': {
                if (--level == 0) {
                    printf("%s\n", line.c_str());
                    line = "";
                }
                break;
            }
            default: {
            }
        }
    }
}

int main(int argc, char *argv[]) {

    int exitValue = 0;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <SCM file>\n", argv[0]);
        exitValue = 1;
    } else {
        std::fstream fin(argv[1], std::fstream::in);
        if (fin.good()) {
            parseFile(fin);
            fin.close();
        } else {
            fprintf(stderr, "Could not open file: \"%s\"", argv[1]);
            exitValue = 1;
        }
    }

    return exitValue;
}
