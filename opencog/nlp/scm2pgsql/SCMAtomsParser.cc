/*
 * SCMAtomsParser.cc
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

#include "SCMAtomsParser.h"

SCMAtomsParser::SCMAtomsParser(const char *fileName)
{

    FILE *f = fopen(fileName);
    if (NULL == f) {
        logger().error("SCMAtomParser() could not open file: \"%s\"", fileName);
    } else {
        logger().info("Parsing \"%s\"", fileName);
        parseFile(f);
        fclose(f);
    }
}

SCMAtomsParser::~SCMAtomsParser()
{
}

void SCMAtomsParser::parseFile(FILE *f)
{
    char *line = NULL;
    size_t lineLength;
    int lineCount;

    enum {
        EXPECT_ATOM_DEFINITION,
        PENDING_LINK_DEFINITION
    } state;

    // Controls the parser state-machine below
    state = EXPECT_ATOM_DEFINITION;

    // Keep track of current link nesting level
    int depth = 0;

    while (getline(&line, &lineLength, f) != -1) {
        lineCount++;
        switch (state) {
            case EXPECT_ATOM_DEFINITION: {
                if (isNodeDefinition(line, lineLength)) {
                } else if (isLinkDefinition(line, lineLength)) {
                }
                break;
            }
            case PENDING_ATOM_DEFINITION: {
                break;
            }
            default: {
            }
        }
    }

    logger().info("Parsed \"%d\" lines", lineCount);

}
