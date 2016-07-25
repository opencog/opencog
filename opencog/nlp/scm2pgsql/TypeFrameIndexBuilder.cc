#include "TypeFrameIndexBuilder.h"
#include <opencog/util/Logger.h>

using namespace opencog;

TypeFrameIndexBuilder::TypeFrameIndexBuilder(TypeFrameIndex *index) 
{
    this->index = index;
}

TypeFrameIndexBuilder::~TypeFrameIndexBuilder() 
{
}

void TypeFrameIndexBuilder::beforeInserting(const std::string &schemeStr)
{
    if (index->addFromScheme(schemeStr)) {
        logger().warn("Error building FrameTypeIndex. Corresponding toplevel atom will not be inserted. Could not parse scheme string: " + schemeStr);
    }
}

void TypeFrameIndexBuilder::afterInserting(const Handle &toplevelAtom)
{
}
