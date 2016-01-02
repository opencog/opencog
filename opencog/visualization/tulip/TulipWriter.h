/*
 * Copyright (C) 2008 by OpenCog Foundation
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

#ifndef _TULIP_WRITER_H
#define _TULIP_WRITER_H

#include <fstream>
#include <iostream>
#include <string>

#include <opencog/atoms/base/Handle.h>

namespace opencog
{
// Caution, will generate invalid tulip files if edges have arity > 2
// or if edges link to other edges
class TulipWriter {
    
    std::ofstream myfile;
    std::string filename;
    //bool writeNode(Handle h);
    //bool writeLink(Handle h);

    std::string getDateString();
    void writeNodes();
    void writeEdges();
    void writeHeader(std::string comment);
    void writeCluster(Handle setLink);
    void writeShapes();
    void writeTruthValue();
    void writeDefaultColouring();
    void writeNodeNames();

public:

    TulipWriter(std::string _filename) : filename(_filename) {};
    ~TulipWriter() {};

    //! Eventually support writing only a certain depth from a node.
    //! At the moment just out put everything
    bool write(Handle seed = Handle::UNDEFINED, int depth = -1,
            Handle setLink = Handle::UNDEFINED);

};

}

#endif // _TULIP_WRITER_H
