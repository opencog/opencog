/** 
 * BasicPredicateUpdater.cc
 * 
 * Author: Carlos Lopes 
 * Copyright(c), 2007
 */
#include "PredefinedProcedureNames.h" 
#include "BasicPredicateUpdater.h"
#include "AtomSpaceUtil.h"
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/TLB.h>
#include <atom_types.h>

using namespace OperationalPetController;
using namespace opencog;

Handle BasicPredicateUpdater::getPredHandle(Handle object, std::string predicateName){
  HandleSeq seq0;
  seq0.push_back(object);
	
  // testing if there is a predicate already
  Handle predicateHandle = atomSpace.getHandle(PREDICATE_NODE, predicateName); 	
  if(predicateHandle == Handle::UNDEFINED){
    MAIN_LOGGER.log(LADSUtil::Logger::FINE, "BasicPredUpdater - Predicate '%s' not found.",
		    predicateName.c_str());		
    return Handle::UNDEFINED;
  }
	
  // testing if there is a list link already
  Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, seq0);
  if(listLinkHandle == Handle::UNDEFINED){
    MAIN_LOGGER.log(LADSUtil::Logger::FINE, "BasicPredUpdater - Obj %s has no ListLink.",
		    atomSpace.getName(object).c_str());	
    return Handle::UNDEFINED;
  }
	
  HandleSeq seq;
  seq.push_back(predicateHandle);
  seq.push_back(listLinkHandle);
		
  std::vector<Handle> allHandles;
  atomSpace.getHandleSet(back_inserter(allHandles), seq, NULL, NULL, 2, EVALUATION_LINK, false);
	
  if(allHandles.size() != 1){
    return Handle::UNDEFINED;
  }
  return *(allHandles.begin());	
}

Handle BasicPredicateUpdater::getHandle(std::string objName){

  HandleSeq objHandle;
  atomSpace.getHandleSet(back_inserter(objHandle), SL_OBJECT_NODE, objName, true);

  // found no handle - ERROR
  if(objHandle.size() < 1){
    MAIN_LOGGER.log(LADSUtil::Logger::ERROR, "BasicPredUpdater - Found no Handle for SpaceMap object %s.",
		    objName.c_str());
    return Handle::UNDEFINED;
  } 

  // found more than one handle - WARNING, return the first one
  // TODO: In this case, it could return the one with the more specific SLObject type.
  else if(objHandle.size() > 1) {
    MAIN_LOGGER.log(LADSUtil::Logger::ERROR, "BasicPredUpdater - Found more than one Handle for SpaceMap object %s. Returning the first one.", objName.c_str());
    unsigned int i;
    for( i = 0; i < objHandle.size( ); ++i ) {
      MAIN_LOGGER.log( LADSUtil::Logger::ERROR, "BasicPredUpdater - %s %i", atomSpace.getName(objHandle[i]).c_str( ), atomSpace.getType(objHandle[i]) );
    } // for

  }

  // found exactly one handle
  return objHandle[0];
}


bool BasicPredicateUpdater::isUpdated(Handle object, std::string predicateName){
			
  if(getPredHandle(object, predicateName) == Handle::UNDEFINED){
    return false;
  }
  return true;	
}

void BasicPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp ){
  MAIN_LOGGER.log(LADSUtil::Logger::WARNING, "BasicPredUpdater - Virtual method. Subclasses should implement it.");	
}
