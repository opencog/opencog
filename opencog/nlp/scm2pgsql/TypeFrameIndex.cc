#include "TypeFrameIndex.h"
#include "TypeFrame.h"

using namespace opencog;

TypeFrameIndex::TypeFrameIndex() 
{
}

TypeFrameIndex::~TypeFrameIndex() 
{
}

bool TypeFrameIndex::addFromScheme(const std::string &txt)
{

    bool returnValue = true;
    TypeFrame frame(txt);
    if (frame.isValid()) {
        for (int i = 0; i < frame.size(); i++) {
            
        }
        returnValue = false;
    }

    return returnValue;
}
