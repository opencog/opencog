#ifndef _OPENCOG_SPACE_SERVER_CONTAINER_WRAP_H
#define _OPENCOG_SPACE_SERVER_CONTAINER_WRAP_H

#include <string>

#include "Handle.h"
#include "SpaceServerContainer.h"
#include <boost/python/wrapper.hpp>

using namespace opencog;
using namespace boost::python;

/** Exposes the SpaceServerContainer class. */
void init_SpaceServerContainer_py();

/** A class wrapper of the SpaceServerContainer class.
 *
 * Dummy classes like these are necessary for wrapping the virtual functions 
 * of classes.
 */
struct SpaceServerContainerWrap : SpaceServerContainer, wrapper<SpaceServerContainer>
{
    // Pure virtual functions.

    void mapRemoved(Handle mapId);
    void mapPersisted(Handle mapId);
    std::string getMapIdString(Handle mapId);
};

#endif // _OPENCOG_SPACE_SERVER_CONTAINER_WRAP_H
