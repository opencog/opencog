#include <string>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>

using namespace std;
namespace opencog
{
	/**
	 *    PredicateNames
	 */
	const string MATERIAL_PREDICATE="material";

	/**
	 *    Utility functions
	 */

	Handle getBlockEntity(const Handle& blockHandle);
	HandleSeq getComposedBlocks(const Handle& blockEntityHandle);
	string getPredicateValue(AtomSpace &atomSpace,
							 string predicateName,Handle a,)
	throw(opencog::NotFoundException);
}
