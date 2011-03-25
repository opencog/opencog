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

std::string NMXmlExporter::toXML(HandleEntry *subset)
{
    HandleSet *exportable = findExportables(subset);

    return(toXML(exportable));
}

std::string NMXmlExporter::toXML(HandleSeq& subset)
{
    HandleEntry* subset_he = HandleEntry::fromHandleVector(subset);
    HandleSet *exportable = findExportables(subset_he);
    return(toXML(exportable));
}

HandleSet *NMXmlExporter::findExportables(HandleEntry *seed)
{
    //Finds the subset.
    HandleSet *exportables = new HandleSet();
    HandleSet *internalLinks  = new HandleSet();
    HandleEntry *it = seed;
    while (it != NULL) {
        exportables->add(it->handle);
        findExportables(exportables, internalLinks, it->handle);
        it = it->next;
    }
    delete(seed);
    //Eliminates internalLinks.
    HandleSetIterator *internals = internalLinks->keys();
    while (internals->hasNext()) {
        Handle h = internals->next();
        exportables->remove(h);
    }
    delete(internals);
    delete(internalLinks);
    return(exportables);
}

void NMXmlExporter::findExportables(HandleSet *exportables, HandleSet *internalLinks, Handle h)
{
    if (!as->isLink(h)) return;

    for (int i = 0; i < as->getArity(h); i++) {
        Handle j = as->getOutgoing(h,i);
        exportables->add(j);

        if (classserver().isA(as->getType(j), LINK)) {
            internalLinks->add(j);
            findExportables(exportables, internalLinks, j);
        }
    }
}

std::string NMXmlExporter::toXML(HandleSet *elements)
{
    bool typesUsed[classserver().getNumberOfClasses()];
    char aux[1<<16];
    std::string result;

    sprintf(aux, "<%s>\n", LIST_TOKEN);
    result += aux;

    memset(typesUsed, 0, sizeof(bool) * classserver().getNumberOfClasses());
    HandleSetIterator *it = elements->keys();
    while (it->hasNext()) {
        exportAtom(it->next(), typesUsed, result);
    }
    delete(it);
    sprintf(aux, "<%s>\n", TAG_DESCRIPTION_TOKEN);
    result += aux;
    for (unsigned int i = 0 ; i < classserver().getNumberOfClasses(); i++) {
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
