#include <iterator>
#include <string>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/util/Logger.h>
#include "AtomSpaceMapUtil.h"

using namespace std;
using namespace opencog;
/* 
   The link structure of ComposedOfLink:
   
   ComposedOfLink
       BlockEntityNode "entityA"
       ListLink
           StructureNode "blockB"
           StructureNode "blockC"
           ...
   In the foloowing block entity query we assume one block only belongs to one block entity. But maybe we will have block belongs to multiple block entity.

*/



Handle getBlockEntity(const Handle& blockHandle, const AtomSpace& atomspace)
{
	HandleSeq listLinks;
	blockHandle->getIncomingSetByType(back_inserter(listLinks), LIST_LINK, false);
	for(auto handle: Listlinks)
	{
		LinkPtr listlink=LinkCast(handle);
		HandleSeq composedOfLink;
		listlink->getIncomingSetByType(back_inserter(composedOfLink),COMPOSED_OF_LINK,false);
		if(!composedOfLink.empty())
		{
			return LinkCast(composedOfLink[0])->getOutgoingAtom(0);
		}
	}
	return Handle::UNDEFINED;
}

HandleSeq getComposedBlocks(const Handle& blockEntityHandle)
{
	HandleSeq composedOfLinks;
	blockEntityHandle->getIncomingSetByType(back_inserter(composedOfLinks), COMPOSED_OF_LINK, false);
	if(composedOfLinks.empty())
	{
		logger().error("getComposedBlock::the block entity handle %u's composedOfLink not found!",blockEntityHandle);
		return HandleSeq();
	}
	//TODO: for now the get_ougoing() in AtomSpace.h is deprecated. But we'd better to have a function to wrap the opearation..
	LinkPtr lll = LinkCast(composedOfLinks[0]);
	Handle listLink=lll->getOutgoingAtom(1);
	return listLink->getOutgoingSet();
}

string getPredicateValue(AtomSpace& atomspace, const string& predicateName,const Handle& blockHandle)
{
	Handle predicateHandle = atomSpace.get_handle(PREDICATE_NODE,predicateName);
    // Create BindLink used by pattern matcher
    std::vector<Handle> listLinkOutgoings,evaluationLinkOutgoings, bindLinkOutgoings;

    Handle hVariableNode = atomspace.add_node(VARIABLE_NODE, "$pred_val");
	listLinkOutgoings.push_back(hVariableNode);
	listLinkOutgoings.push_back(blockHandle);
	Handle hListLink = atomspace.add_link(LIST_LINK,listLinkOutgoings);
	evaluationLinkOutgoings.push_back(predicateHandle);
	evaluationLinkOutgoings.push_back(hListlink);
	Handle hEvaluationLink = atomspace.add_link(EVALUATION_LINK,evaluationLinkOutgoings);
    bindLinkOutgoings.push_back(hVariableNode);
    bindLinkOutgoings.push_back(hEvaluationLink);
    Handle hBindLink = atomspace.add_link(BIND_LINK, bindLinkOutgoings);
    Handle hResultListLink = bindlink(&atomspace, hBindLink);

    Handle result = (LinkCast(hResultListLink)->getOutgoingSet())[0];
    atomspace.remove_atom(hResultListLink);
	return result
}
