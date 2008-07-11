#ifndef _XMLNODELOADER_H_
#define _XMLNODELOADER_H_

namespace reasoning
{
	
	class XMLNode;

Handle Add1NodeFromXML(iAtomTableWrapper* table,
					   const XMLNode& xml,
					   const set<std::string>& old_free_names,
					   set<std::string>& new_free_names,
					   std::map<std::string, std::string>& newVarName);
Handle HandleXMLInputNode(iAtomTableWrapper* table,const XMLNode& xml, const set<std::string>& prev_free_names,
						  std::map<std::string, std::string>& returning_names, std::map<std::string, std::string> newVarName);
Handle Add1LinkFromXML(iAtomTableWrapper* table,const XMLNode& xml, HandleSeq& children);
Handle LoadXMLInput(iAtomTableWrapper* table,std::string buf);
Handle LoadXMLFile(iAtomTableWrapper* table, std::string fname);

#define FRESHNESS_BY_DEFAULT false


} //namespace reasoning

#endif //_XMLNODELOADER_H_
