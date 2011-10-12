/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifdef _MSC_VER
#pragma warning( disable : 4786)
#pragma warning( disable : 4503)
#endif // _MSC_VER

#include "XMLNode.h"
#include "XMLNodeLoader.h"
#include "../AtomSpaceWrapper.h"
#include "../rules/Rules.h"

using namespace opencog;
#define ELEMENT_NODE 1000

void ct();

namespace opencog { namespace pln
{
std::map<nocase_string, int> name2type;
//std::map<int, string> type2name;
std::map<std::string, Handle> nodenames;

int xml_level = -1;
    
Handle LoadXMLFile(iAtomSpaceWrapper* table, string fname)
{
 cprintf(5, "LoadXMLfile...");
    /// \todo haxx::
    xml_level = -1;
    nodenames.clear();
 cprintf(5, "thms clear...");
    CrispTheoremRule::thms.clear();
  cprintf(5, "LoadTextFile...");
    string buf;
    LoadTextFile(fname.c_str(), buf);
  cprintf(5, "LoadXMLInput...");
    return LoadXMLInput(table, buf);
}

Handle LoadXMLInput(iAtomSpaceWrapper* table, string buf)
{
#define L(A, B) name2type[A] = B; type2name[B] = A;

//currentDebugLevel = 5; 
LOG(1, "Loading XML Input...");

    // Use all standard atom type codes and names from ClassServer.h 
    L(classserver().getTypeName(CONCEPT_NODE), CONCEPT_NODE);
    L(classserver().getTypeName(WORD_NODE), WORD_NODE);
    L(classserver().getTypeName(TAIL_PREDICTIVE_IMPLICATION), TAIL_PREDICTIVE_IMPLICATION);
    L(classserver().getTypeName(PREDICTIVE_IMPLICATION), PREDICTIVE_IMPLICATION);
    L(classserver().getTypeName(EVENTUAL_PREDICTIVE_IMPLICATION_LINK), EVENTUAL_PREDICTIVE_IMPLICATION_LINK);
    L(classserver().getTypeName(SIMULTANEOUS_AND_LINK), SIMULTANEOUS_AND_LINK);
    L(classserver().getTypeName(SEQUENTIAL_AND_LINK), SEQUENTIAL_AND_LINK);
    L(classserver().getTypeName(EVENTUAL_SEQUENTIAL_AND_LINK), EVENTUAL_SEQUENTIAL_AND_LINK);
    L(classserver().getTypeName(SATISFYING_SET_LINK), SATISFYING_SET_LINK);
    L(classserver().getTypeName(SCHEMA_NODE), SCHEMA_NODE);
    L(classserver().getTypeName(PREDICATE_NODE), PREDICATE_NODE);
    L(classserver().getTypeName(VARIABLE_NODE), VARIABLE_NODE);
    L(classserver().getTypeName(EXTENSIONAL_INHERITANCE_LINK), EXTENSIONAL_INHERITANCE_LINK);
    L(classserver().getTypeName(INHERITANCE_LINK), INHERITANCE_LINK);
    L(classserver().getTypeName(EVALUATION_LINK), EVALUATION_LINK);
    L(classserver().getTypeName(LIST_LINK), LIST_LINK);
    L(classserver().getTypeName(IMPLICATION_LINK), IMPLICATION_LINK);
    L(classserver().getTypeName(AND_LINK), AND_LINK);
    L(classserver().getTypeName(OR_LINK), OR_LINK);
    L(classserver().getTypeName(NOT_LINK), NOT_LINK);
    L(classserver().getTypeName(EXISTS_LINK), EXISTS_LINK);
    L(classserver().getTypeName(FORALL_LINK), FORALL_LINK);
    L(classserver().getTypeName(EQUIVALENCE_LINK), EQUIVALENCE_LINK);
    L(classserver().getTypeName(EXECUTION_OUTPUT_LINK), EXECUTION_OUTPUT_LINK);
    L(classserver().getTypeName(EXECUTION_LINK), EXECUTION_LINK);
    L(classserver().getTypeName(MEMBER_LINK), MEMBER_LINK);
    L(classserver().getTypeName(SUBSET_LINK), SUBSET_LINK);
    L(classserver().getTypeName(SCHOLEM_LINK), SCHOLEM_LINK);
    L(classserver().getTypeName(FALSE_LINK), FALSE_LINK);
    L(classserver().getTypeName(TRUE_LINK), TRUE_LINK);
    L(classserver().getTypeName(HYPOTHETICAL_LINK), HYPOTHETICAL_LINK);

    // Alternative names for atom types used in xml files.
    L("TPC",TAIL_PREDICTIVE_IMPLICATION);
    L("EventualPredictiveImplication",EVENTUAL_PREDICTIVE_IMPLICATION_LINK);
    L("SimAND",SIMULTANEOUS_AND_LINK);
    L("SeqAND",SEQUENTIAL_AND_LINK);
    L("EvSeqAND",EVENTUAL_SEQUENTIAL_AND_LINK);
    L("SatisfyingSet",SATISFYING_SET_LINK);
    L("SatSet",SATISFYING_SET_LINK);
    L("FreeVariableNode", FW_VARIABLE_NODE);
    L("FW_VariableNode", FW_VARIABLE_NODE);
    L("FWVariableNode", FW_VARIABLE_NODE);
    // L("ExtInhLink", EXTENSIONAL_INHERITANCE_LINK);
    L("And", AND_LINK);
    L("Or", OR_LINK);
    L("Not", NOT_LINK);
    L("Exist", EXISTS_LINK);
    L("ForAll", FORALL_LINK);
    L("ExOutLink", EXECUTION_OUTPUT_LINK);
    L("SFunctionLink", SCHOLEM_LINK);
    L("False", FALSE_LINK);
    L("True", TRUE_LINK);
    L("TOPOLOGICAL", HYPOTHETICAL_LINK); //Should never occur?
    L("HYPOTHETICAL", HYPOTHETICAL_LINK); //Should never occur?

    // Other tokens used in xml files
    // WARNING: THE NUMERIC CODES USED FOR THESE TOKENS MAY CONFLICT WITH ATOM TYPE CODES IN THE FUTURE
    L("Element", ELEMENT_NODE); // defined in this file
    L("__OR", __OR); // defined in PLNatom.h
    L("__AND", __AND); // defined in PLNatom.h
    L("__NOT", __NOT); // defined in PLNatom.h
    L("R", RESTRICTOR); // defined in PLNatom.h
    L("InstanceOf", INSTANCEOF_R); // defined in PLNatom.h
    L("In", IN_R); // defined in PLNatom.h
    L("Contains", HAS_R); // defined in PLNatom.h
    
    // Associative Link
    L("AssociativeLink", ASSOCIATIVE_LINK); // defined in PLNatom.h

    set<string> free_names;
    std::map<string, string>  returning_names;
    std::map<string, string> newVarName;

    std::vector<Handle> hs;

    XMLNode xml(buf);
    assert(xml.ok); 

    return HandleXMLInputNode(table, xml, free_names, returning_names, newVarName);
}

//  name2type[""] = ;

set<string> declared_vars;
/*set<string> vars_in_scope;
stack< set<string> > scope;*/

bool valid(Type T, HandleSeq& children)
{
    //if (PLN_CONFIG_COLLAPSE_LIST_LINKS)
    //{
    //  if (inheritsType(T, LIST_LINK) && children.empty())
    //      return false;
    //}
    return true;
}

TruthValue* CreateTVfromArguments(const XMLNode& xml)
{
    std::map<nocase_string,nocase_string> arguments = xml.TagData().arguments;

    float s = (STLhas(arguments,"strength") ? atof(arguments["strength"].c_str()) : 1.0f);
    float c = (STLhas(arguments,"confidence") ? atof(arguments["confidence"].c_str()) : 0.0f);

    TruthValue* tv = new SimpleTruthValue(s,SimpleTruthValue::confidenceToCount(c));
//cprintf(3, "Confidence=%f\n",c);  
    return tv;
}

Handle Add1NodeFromXML(iAtomSpaceWrapper* table,
                        const XMLNode& xml,
                       const set<string>& old_free_names,
                       set<string>& new_free_names,
                       std::map<string, string>& newVarName)
{
    Handle ret;

    for (std::map<string, string>::iterator j = newVarName.begin(); j != newVarName.end(); j++)
        LOG(5, j->first + " |=> " + j->second);

    std::map<nocase_string,nocase_string> arguments = xml.TagData().arguments;

    Type root_type = name2type[xml.TagData().name];
    if (root_type == ELEMENT_NODE)
        root_type = name2type[arguments["class"]];

    const bool isVar = GET_ASW->inheritsType( root_type, VARIABLE_NODE);

    if (STLhas(nodenames, arguments["name"])
        && !isVar)
    {
        LOG(4, "Replica:"+ arguments["name"]);
        return nodenames[arguments["name"]];
    }

    string name = arguments["name"];

    if (isVar)
    {   
        if (STLhas(newVarName, name)) //Taken, conversion known
            name = newVarName[name];
        else if (STLhas(declared_vars, name) ) //Taken, NO conversion defined on this scope
        {
            string original_name = name;
            
            while (STLhas(declared_vars, name))
            {
LOG(5, "...VAR_CREATION");
                name = "$"+GetRandomString(10);
            }
            
            newVarName[original_name] = name;
            new_free_names.insert(name);
        }
        else                                //New name introduced, ok.
        {
            new_free_names.insert(name);
            newVarName[name] = name;
        }
    }

    declared_vars.insert(name);

    TruthValue *tvn = CreateTVfromArguments(xml);

    LOG(4, "Adding " + name);
    LOG(4, i2str(tvn->getMean()*1000) + " / " + i2str(tvn->getConfidence()));

    ret = table->addNode(root_type, name,
        *tvn, false); //isVar);
        
    delete tvn;

    nodenames[name] = ret;

    LOG(4, "Added Node: " + name);

    return ret;
}

Handle Add1LinkFromXML(iAtomSpaceWrapper* table, const XMLNode& xml, HandleSeq& children)
{
    Handle ret;
LOG(5, "Add1LinkFromXML...");
    Type root_type = name2type[xml.TagData().name];
    
    if (!root_type)
    {
        LOG(1, "Type lookup failed for " + xml.TagData().name);
    }   

    if (!valid(root_type, children))
        return Handle::UNDEFINED;

    TruthValue* tvn = CreateTVfromArguments(xml);
    if (children.size() <= 7)
        ret = table->addLink(root_type, children, *tvn, false);
    else
        ret=Handle::UNDEFINED;
    delete tvn;

LOG(5, "Add1LinkFromXML OK!");

    return ret;
}

//set<string> last_level_scoped_names;

static int helper1=0, helper2=0, helper3=0;

Handle HandleXMLInputNode(iAtomSpaceWrapper* table, const XMLNode& xml, const set<string>& prev_free_names,
                          std::map<string, string>& returning_names, std::map<string, string> newVarName)
{
    Handle ret;
    HandleSeq children;
    returning_names.clear();

    xml_level++;

    set<string> this_level_scoped_names;
    set<string> all_scoped_names;

    if (xml.TagData().name == "note")
    {
LOG(5, "NOTE tag detected");
    }
    if (xml.Sub().empty())// || xml.TagData().name == "note")
    {
LOG(5,"If: Add1NodeFromXML");
        ret = Add1NodeFromXML(table, xml, prev_free_names, this_level_scoped_names, newVarName);
    }
    else
    {
LOG(5,"XMLITERATE:");
        XMLITERATE(xml, child1)
        {
            all_scoped_names.clear();
            all_scoped_names = this_level_scoped_names;

            LOG(5, "---");
            //for (std::map<string, string>::iterator j = newVarName.begin(); j != newVarName.end(); j++)
            //  LOG(5, j->first + " => " + j->second + "\n");

            for (set<string>::const_iterator s2 = prev_free_names.begin();
                        s2!=prev_free_names.end(); s2++)
                all_scoped_names.insert(*s2);

            if (!(*child1)->Sub().empty())
            {
LOG(5,"HandleXMLInputNodexx:");
                Handle new_node = HandleXMLInputNode(table, **child1, all_scoped_names, returning_names, newVarName);
                /// A special treatment is given to Listed ForAlls. Others, just add to da std::vector.
                if (   !GET_ASW->isSubType(new_node, LIST_LINK)
                    || GET_ASW->getArity(new_node) <= 0
                    || !GET_ASW->isSubType(GET_ASW->getOutgoing(new_node,0), FORALL_LINK))
                {
                    children.push_back(new_node);
helper2=5;
                }
                else
                {
                    for (int fl=0;fl<GET_ASW->getArity(new_node);fl++)
                        children.push_back(GET_ASW->getOutgoing(new_node, fl));
helper2=3;
                }
                /// The next level's names will be forgotten, unless next level was ListLink.
                for (std::map<string, string>::iterator s3 = returning_names.begin();
                    s3 != returning_names.end(); s3++)
                    {
                        newVarName[s3->first] = s3->second;

                        this_level_scoped_names.insert(s3->first);
                    }

                returning_names.clear();
            }
            else
            {
if (++helper3 == 1069)
    puts("1069");
LOG(5,"else: Add1NodeFromXML");
                Handle h1 = Add1NodeFromXML(table, **child1, prev_free_names, this_level_scoped_names, newVarName);
                children.push_back(h1);
helper2 = 2;
            }
        }
LOG(5,"Add1LinkFromXML");

if (++helper1 == 775)
    puts("775");
            ret = Add1LinkFromXML(table, xml, children);            
    }

    //if (TheNM.inheritsType(getType(ret), LIST_LINK))
    if (xml_level != 1)
        returning_names = newVarName;

    xml_level--;
LOG(5,"XML:returnxx");
    return ret;
}

}} //~namespace opencog::pln
