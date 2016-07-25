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
    TypeFrame frame(txt);
    return ! frame.isValid();
}
