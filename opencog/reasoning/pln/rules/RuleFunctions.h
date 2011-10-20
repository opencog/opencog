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

#ifndef RULEFUNCTION_H
#define RULEFUNCTION_H

using namespace opencog;

Handle child(Handle h, int i);

namespace opencog { namespace pln {

#ifndef WIN32
//! Max of two floats
float max(float a, float b);
#endif

/** Create a new FWVAR node with the given name.
 * @relates AtomSpaceWrapper
 */
Vertex CreateVar(iAtomSpaceWrapper* asw, const std::string& varname);

/** Create a new FWVAR node and generate the name.
 * @relates AtomSpaceWrapper
 */
Vertex CreateVar(iAtomSpaceWrapper* asw);

Rule::setOfMPs makeSingletonSet(const Rule::MPs& mp);

BBvtree atomWithNewType(Handle h, Type T, AtomSpaceWrapper* asw);
BBvtree atomWithNewType(const tree<Vertex>& v, Type T, AtomSpaceWrapper* asw);
BBvtree atomWithNewType(const Vertex& v, Type T, AtomSpaceWrapper* asw);	

bool UnprovableType(Type T);

Rule::setOfMPs PartitionRule_o2iMetaExtra(meta outh, bool& overrideInputFilter,
                                          Type OutLinkType,
                                          AtomSpaceWrapper* asw);

/**
 * FindMatchingUniversals retrieves all FORALL_LINK,
 * put then in the global variable ForAll_handles and then run
 * FindMatchingUniversal on each of them
 */
Btr< std::set<Btr<ModifiedBoundVTree> > > FindMatchingUniversals(Btr<vtree> target,
                                                                 AtomSpaceWrapper* asw);
Btr<ModifiedBoundVTree> FindMatchingUniversal(meta target,
                                              pHandle ForAllLink,
                                              AtomSpaceWrapper* asw);
	
Handle And2OrLink(Handle& andL, Type _AndLinkType, Type _OR_LINK);
Handle Or2AndLink(Handle& andL);
Handle And2OrLink(Handle& andL);
Handle Exist2ForAllLink(Handle& exL);
std::pair<Handle,Handle> Equi2ImpLink(Handle&);
#define LINKTYPE_ASSERT(__cLink, __cLinkType) assert(inheritsType(GET_ASW->getType(__cLink), __cLinkType))

}} // namespace opencog { namespace pln {
#endif // RULEFUNCTION_H
