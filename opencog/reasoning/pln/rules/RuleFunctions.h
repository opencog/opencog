#ifndef RULEFUNCTION_H
#define RULEFUNCTION_H

using namespace opencog;

Handle child(Handle h, int i);
bool isSubType(Handle h, Type T);
bool inheritsType(Type T1, Type T2);

namespace reasoning
{

#ifndef WIN32
  float max(float a, float b);
#endif

Vertex CreateVar(iAtomSpaceWrapper* atw, std::string varname);
Vertex CreateVar(iAtomSpaceWrapper* atw);

Rule::setOfMPs makeSingletonSet(Btr<Rule::MPs> mp);

BBvtree atomWithNewType(Handle h, Type T);
BBvtree atomWithNewType(const tree<Vertex>& v, Type T);
BBvtree atomWithNewType(const Vertex& v, Type T);	

bool UnprovableType(Type T);

Rule::setOfMPs PartitionRule_o2iMetaExtra(meta outh, bool& overrideInputFilter, Type OutLinkType);
	
Handle AND2ORLink(Handle& andL, Type _ANDLinkType, Type _OR_LINK);
Handle OR2ANDLink(Handle& andL);
Handle AND2ORLink(Handle& andL);
Handle Exist2ForAllLink(Handle& exL);
pair<Handle,Handle> Equi2ImpLink(Handle&);
#define LINKTYPE_ASSERT(__cLink, __cLinkType) assert(inheritsType(GET_ATW->getType(__cLink), __cLinkType))

} // namespace reasoning
#endif // RULEFUNCTION_H
