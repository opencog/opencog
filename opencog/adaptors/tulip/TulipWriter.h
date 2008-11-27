#ifndef _TULIP_WRITER_H
#define _TULIP_WRITER_H

#include <opencog/atomspace/types.h>

#include <iostream>
#include <fstream>

namespace opencog {
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
