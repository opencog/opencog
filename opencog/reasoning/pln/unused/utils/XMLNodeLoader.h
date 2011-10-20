/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _XMLNODELOADER_H_
#define _XMLNODELOADER_H_

namespace opencog { namespace pln
{
	
	class XMLNode;

Handle Add1NodeFromXML(iAtomSpaceWrapper* table,
					   const XMLNode& xml,
					   const set<std::string>& old_free_names,
					   set<std::string>& new_free_names,
					   std::map<std::string, std::string>& newVarName);
Handle HandleXMLInputNode(iAtomSpaceWrapper* table,const XMLNode& xml, const set<std::string>& prev_free_names,
						  std::map<std::string, std::string>& returning_names, std::map<std::string, std::string> newVarName);
Handle Add1LinkFromXML(iAtomSpaceWrapper* table,const XMLNode& xml, HandleSeq& children);
Handle LoadXMLInput(iAtomSpaceWrapper* table,std::string buf);
Handle LoadXMLFile(iAtomSpaceWrapper* table, std::string fname);

#define FRESHNESS_BY_DEFAULT false


} //namespace opencog::pln

#endif //_XMLNODELOADER_H_
