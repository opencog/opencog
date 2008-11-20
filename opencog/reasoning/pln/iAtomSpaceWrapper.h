#ifndef IATOMTABLEWRAPPER_H
#define IATOMTABLEWRAPPER_H

#include "AtomLookupProvider.h"

namespace reasoning
{

struct iAtomSpaceWrapper : public ::AtomLookupProvider
{
	virtual Handle addAtom(tree<Vertex>&, const TruthValue& tvn,
            bool fresh, bool managed = true)=0;
	virtual Handle addLink(Type T, const HandleSeq& hs,
            const TruthValue& tvn, bool fresh=false, bool managed = true)=0;
	virtual Handle addNode(Type T, const std::string& name,
            const TruthValue& tvn, bool fresh=false, bool managed = true)=0;

	virtual unsigned int getUniverseSize() const=0;
};

}

#endif
