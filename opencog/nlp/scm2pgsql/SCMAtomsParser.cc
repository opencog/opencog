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

    int lineAllocSize = 1024;
    char *currentLine = (char *) malloc(lineAllocSize);
    int cursor = 0;
    currentLine[0] = '\0';

    while ((int c = fgetc(f)) != EOF) {
        if (cursor == (lineAllocSize - 1)) {
            lineAllocSize = 2 * lineAllocSize;
            currentLine = (char *) realloc(currentLine, lineAllocSize);
        }
        currentLine[cursor++] = c;
        currentLine[cursor] = '\0';
        switch (c) {
            case '(': {
                level++;
                break;
            }
            case ')': {
                if (--level == 0) {
                    printf("%s\n" currentLine);
                    currentLine[0] = '\0';
                }
                break;
            }
            default: {
                // do nothing
            }
        }
    }
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
                Type atomType = parseAtomType(line);
                if (classserver().isNode(atomType)) {
                    std::string nodeName = parseNodeName(line);
                    Handle handle = getNodeHandle(atomType, nodeName);
                    AtomPtr node(createNode(atomType, nodeName));
                    TLB::addAtom(node);
                } else if (classserver().isNode(atomType)) {
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
    if (NULL != line) {
        free(line);
    }
}

Handle SCMAtomsParser::getNodeHandle(Type type, std::string name)
{

}

Type SCMAtomsParser::parseAtomType(char *line)
{
    
    Type answer = NOTYPE;

    char *cursor1 = strchr(s, '(');
    if (NULL != cursor1) {
        char *cursor2 = strchr(s, ' ');
        if (NULL != cursor2) {
            int length = cursor2 - cursor1 - 1;
            std::string typeName(cursor1 + 1, length);
            answer = classserver().getType(typeName);
        }
    }

    return answer;
}

std::string SCMAtomsParser::parseNodeName(char *line)
{
    
    // TODO: This method will fail if " is used in the Node's name.
    // For example: (WordNode "\"WeirdNodeName\"")
      
    std::string answer = "";

    char *cursor1 = strchr(s, '\"');
    if (NULL != cursor1) {
        char *cursor2 = strchr(cursor1 + 1, '\"');
        if (NULL != cursor2) {
            int length = cursor2 - cursor1 - 1;
            std::string nodeName(cursor1 + 1, length);
            //answer = nodeName;
            return nodeName;
        }
    }

    return answer;
}
