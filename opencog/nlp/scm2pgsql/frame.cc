/*
 * frame.cc
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

#include "TypeFrame.h"
#include <opencog/guile/SchemeEval.h>

using namespace opencog;
//using namespace opencog::nlp;

bool loadFile(char *fileName)
{
    /*
    TypeFrame *tf = new TypeFrame("\
(t1 \
(t2 (stv 1 1) \
(t3 \"ola\") \
(t3 \"ole\" (stv 0.07692308 0.0012484394)) \
) \
(t2 (stv 1 1) \
(t3 \"oli\") \
(t3 \"olo\" (stv 1 0.3312)) \
) \
(t2 (stv 1 1) \
(t3 \"olu\") \
(t3 \"yep\") \
) \
) \
");

    TypeFrame *tf2 = new TypeFrame("(t1 (t2 (t3 \"ola\") (t3 \"ola\")) (t3    \"ola\"))");
    */

    return true;

}

int main(int argc, char *argv[]) {

    int exitValue = 0;

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <SCM file>\n", argv[0]);
        exitValue = 1;
    } else {
        if (! loadFile(argv[1])) {
            exitValue = 1;
        }
    }

    return exitValue;
}
