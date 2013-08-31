/*
 * opencog/xml/NMXmlExporter.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Rodrigo Barra
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

#include "NMXmlExporter.h"

#include <opencog/util/platform.h>

#include <opencog/atomspace/Link.h>
#include <opencog/persist/xml/NMXmlDefinitions.h>

using namespace opencog;

NMXmlExporter::NMXmlExporter(const AtomSpace* _as) :
    as(_as)
{
}

NMXmlExporter::~NMXmlExporter() {
}

std::string NMXmlExporter::toXML(const HandleSeq& subseq)
{
    UnorderedHandleSet subset;
    std::copy(subseq.begin(), subseq.end(), inserter(subset));
    UnorderedHandleSet *exportable = findExportables(subset);

    return(toXML(exportable));
}

#if 0
std::string NMXmlExporter::toXML(const UnorderedHandleSet* subset)
{
    UnorderedHandleSet *exportable = findExportables(*subset);
    return(toXML(exportable));
}
#endif

UnorderedHandleSet *NMXmlExporter::findExportables(const UnorderedHandleSet& seed)
{
    //Finds the subset.
    UnorderedHandleSet *exportables = new UnorderedHandleSet();
    UnorderedHandleSet *internalLinks  = new UnorderedHandleSet();
    UnorderedHandleSet::const_iterator it;
    for (it = seed.begin(); it != seed.end(); it++) {
        exportables->insert(*it);
        findExportables(exportables, internalLinks, *it);
    }
    //Eliminates internalLinks.
    UnorderedHandleSet::iterator internals = internalLinks->begin();
    while (internals != internalLinks->end()) {
        Handle h = *internals;
        internals++;
        exportables->erase(h);
    }
    delete(internalLinks);
    return(exportables);
}

void NMXmlExporter::findExportables(UnorderedHandleSet *exportables, UnorderedHandleSet *internalLinks, Handle h)
{
    if (!as->isLink(h)) return;

    for (int i = 0; i < as->getArity(h); i++) {
        Handle j = as->getOutgoing(h,i);
        exportables->insert(j);

        if (classserver().isA(as->getType(j), LINK)) {
            internalLinks->insert(j);
            findExportables(exportables, internalLinks, j);
        }
    }
}

std::string NMXmlExporter::toXML(const UnorderedHandleSet *elements)
{
    bool typesUsed[classserver().getNumberOfClasses()];
    char aux[1<<16];
    std::string result;

    sprintf(aux, "<%s>\n", LIST_TOKEN);
    result += aux;

    memset(typesUsed, 0, sizeof(bool) * classserver().getNumberOfClasses());
    UnorderedHandleSet::const_iterator it = elements->begin();
    while (it != elements->end()) {
        exportAtom(*it, typesUsed, result);
        it++;
    }
    sprintf(aux, "<%s>\n", TAG_DESCRIPTION_TOKEN);
    result += aux;
    unsigned int numberOfTypes = classserver().getNumberOfClasses();
    for (unsigned int i = 0 ; i < numberOfTypes; i++) {
        if (typesUsed[i]) {
            sprintf(aux, "<%s %s=\"%s\" %s=\"%s\" />\n", TAG_TOKEN, NAME_TOKEN, classserver().getTypeName(i).c_str(), VALUE_TOKEN, classserver().getTypeName(i).c_str());
            result += aux;
        }
    }
    sprintf(aux, "</%s>\n", TAG_DESCRIPTION_TOKEN);
    result += aux;

    sprintf(aux, "</%s>\n", LIST_TOKEN);
    result += aux;

    delete(elements);
    return(result);
}


void NMXmlExporter::exportAtom(Handle atomHandle, bool typesUsed[], std::string& result, bool isInternal)
{
    //printf("Exporting %s\n", as->atomAsString(atomHandle).c_str());
    char aux[1<<16];
    Type t = as->getType(atomHandle);
    typesUsed[t] = true;
    TruthValuePtr tv = as->getTV(atomHandle);
    if (classserver().isA(t, NODE)) {
        if (!isInternal) {
            sprintf(aux, "<%s %s=\"%f\" %s=\"%f\" ",
                    classserver().getTypeName(t).c_str(),
                    STRENGTH_TOKEN,
                    tv->getMean(),
                    CONFIDENCE_TOKEN,
                    tv->getConfidence());
            result += aux;
        } else {
            sprintf(aux, "<%s %s=\"%s\" ", ELEMENT_TOKEN, CLASS_TOKEN, classserver().getTypeName(t).c_str());
            result += aux;
        }
        std::string name = as->getName(atomHandle);
        if (name == "")
            name = "#" + atomHandle;
        sprintf(aux, "%s=\"%s\" />\n", NAME_TOKEN, name.c_str());
        result += aux;
    } else {
        sprintf(aux, "<%s %s=\"%f\" %s=\"%f\" ",
                classserver().getTypeName(t).c_str(),
                STRENGTH_TOKEN,
                tv->getMean(),
                CONFIDENCE_TOKEN,
                tv->getConfidence());
        result += aux;
        sprintf(aux, ">\n");
        result += aux;

        if (as->isLink(atomHandle)) {
            for (int i = 0; i < as->getArity(atomHandle); i++) {
                exportAtom(as->getOutgoing(atomHandle,i), typesUsed, result, true);
            }
        }
        sprintf(aux, "</%s>\n", classserver().getTypeName(t).c_str());
        result += aux;
    }

}
