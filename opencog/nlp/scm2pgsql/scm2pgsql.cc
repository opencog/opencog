/*
 * scm2pgsql.cc
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

#include "PGStorageDelegate.h"

using namespace opencog;
using namespace opencog::nlp;

int main(int argc, char *argv[]) {

    int exitValue = 0;

    if (argc != 5) {
        fprintf(stderr, "Usage: %s <SCM file> <DB name> <User name> <Passord>\n", argv[0]);
        exitValue = 1;
    } else {
        PGStorageDelegate storageDelegate(argv[2], argv[3], argv[4]);
        exitValue = storageDelegate.loadSCMFile(argv[1]);
    }

    return exitValue;
}
