/*
 * src/AtomSpace/SavingLoading.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

/* SavingLoading.cc - Saves/loads the atom network (or a subset of it) to/from
 * disk */

#include "platform.h"

#include "SavingLoading.h"
#include "ClassServer.h"
#include "HandleIterator.h"
#include "Link.h"
#include "Node.h"
#include "TLB.h"
#include "types.h"
#include "CoreUtils.h"
#include "StatisticsMonitor.h"
#include "CompositeTruthValue.h"
#include "AtomSpaceDefinitions.h"
#include "Logger.h"
#include "HandleMap.cc"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace opencog;

//#include "../agents/DynamicsStatisticsAgent.h"
//#include "TreePredicateEvaluator.h"
//#include "NMPrinter.h"

#define FULL_NETWORK_DUMP          (1 << 0)
#define ATOM_SET                   (1 << 1)
#define PHYSICAL_ADDRESSING        (1 << 2)

#define INDEX_REPORT_FACTOR             1.02
#define POST_PROCESSING_REPORT_FACTOR   1.10

int processed = 0;
int total = 0;

SavingLoading::SavingLoading()
{
}

void SavingLoading::save(const char *fileName, AtomSpace& atomSpace) throw (IOException)
{

    logger().info("Starting Memory dump");

    time_t start = time(NULL);

    // opens the file that will be modified
    FILE *f = fopen(fileName, "wb");
    if (f == NULL) {
        throw IOException(TRACE_INFO,
                          "SavingLoading - Unable to open file '%s' for writing", fileName);
    }

    AtomTable& atomTable = atomSpace.atomTable;

    // stores the total number of atoms in the system
    int atomCount = atomTable.getSize();

    // a flag that indicates the file format
    char format = FULL_NETWORK_DUMP;

    // writes on the file the file format and the  number of atoms
    fwrite(&format, sizeof(char), 1, f);
    fwrite(&atomCount, sizeof(int), 1, f);

    processed = 0;
    total = atomCount;

    // save classserver info
    saveClassServerInfo(f);

    saveNodes(f, atomTable, atomCount);
    saveLinks(f, atomTable, atomCount);
    saveIndices(f, atomTable);

    atomSpace.timeServer.saveRepository(f);
    saveRepositories(f);

    // closes the file
    fclose(f);

    // calculates the total time that the process of saving has spent
    time_t duration = time(NULL) - start;
    logger().info("Memory dump: 100%% done (in %d second%c).",
                  (int) duration, duration == 1 ? '\0' : 's');
    fflush(stdout);
}

void SavingLoading::saveClassServerInfo(FILE *f)
{
    logger().fine("SavingLoading::saveClassServerInfo");
    int numTypes = ClassServer::getNumberOfClasses();

    fwrite(&numTypes, sizeof(int), 1, f);

    for (Type i = 0; i < numTypes; i++) {
        int classNameLength = ClassServer::getTypeName(i).length();
        fwrite(&classNameLength, sizeof(int), 1, f);
        fwrite(ClassServer::getTypeName(i).c_str(), sizeof(char), classNameLength, f);
        fwrite(&i, sizeof(Type), 1, f);
    }
}

void SavingLoading::saveNodes(FILE *f, AtomTable& atomTable, int &atomCount)
{
    logger().fine("SavingLoading::saveNodes");

    int numNodes = 0;

    // gets the position of the pointer on the file for future reference
    int numNodesOffset = ftell(f);

    // writes 0 on the file. Later, the total number of nodes will be written
    // here
    fwrite(&numNodes, sizeof(int), 1, f);

    // creates an iterator to iterate on all nodes
    HandleIterator* iter = atomTable.getHandleIterator(NODE, true);

    // writes nodes to file and increments node counter
    while (iter->hasNext()) {
        Handle atomHandle = iter->next();
        Node* node = (Node*) TLB::getAtom(atomHandle);
        writeNode(f, node);
        numNodes++;
        int percentage = (int) (100 * ((float) ++processed / (total * INDEX_REPORT_FACTOR)));
        if ((percentage % 10) == 0) {
            printf( "Memory dump: %d%% done.\r", percentage);
            fflush(stdout);
        }
    }

    delete iter;

    // rewind to position where number of nodes must be written
    fseek(f, numNodesOffset, SEEK_SET);

    // writes the number of nodes in the proper position
    fwrite(&numNodes, sizeof(int), 1, f);

    //updates atomCount
    atomCount += numNodes;

    // returns the pointer back to the end of the file
    fseek(f, 0, SEEK_END);
}

void SavingLoading::saveLinks(FILE *f, AtomTable& atomTable, int &atomCount)
{
    logger().fine("SavingLoading::saveLinks");

    int numLinks = 0;
    // gets the position of the pointer on the file for future reference
    int numLinksOffset = ftell(f);

    // writes 0 on the file. Later, the total number of links will be written
    // here
    fwrite(&numLinks, sizeof(int), 1, f);

    // creates a iterator to iterate on all links
    HandleIterator* iter = atomTable.getHandleIterator(LINK, true);

    // writes links to file and increments link counter
    while (iter->hasNext()) {
        Handle atomHandle = iter->next();
        Link* link = (Link*) TLB::getAtom(atomHandle);
        writeLink(f, link);
        numLinks++;
        printf( "Memory dump: %d%% done.\r", (int) (100 * ((float) ++processed / (total * INDEX_REPORT_FACTOR))));
        fflush(stdout);
    }

    delete iter;

    // rewind to position where number of links must be written
    fseek(f, numLinksOffset, SEEK_SET);

    // writes the number of links in the proper position
    fwrite(&numLinks, sizeof(int), 1, f);

    // updates the atomCount
    atomCount += numLinks;

    // returns the pointer back to the end of the file
    fseek(f, 0, SEEK_END);
}


void SavingLoading::saveIndices(FILE *f, AtomTable& atomTable)
{
    logger().fine("SavingLoading::saveIndices");

    int numTypes = ClassServer::getNumberOfClasses();

    // writes the head of each index list on the file
    for (int i = 0; i < numTypes; i++) {
        fwrite(&(atomTable.typeIndex[i]), sizeof(Handle), 1, f);
    }
    printf( "Memory dump: %d%% done.\r", (int) (100 *
            (((float) processed + (0.25 * ((total * INDEX_REPORT_FACTOR) - processed)))
             / (total * INDEX_REPORT_FACTOR))));
    fflush(stdout);

    for (int i = 0; i < numTypes; i++) {
        fwrite(&(atomTable.targetTypeIndex[i]), sizeof(Handle), 1, f);
    }
    printf( "Memory dump: %d%% done.\r", (int) (100 *
            (((float) processed + (0.50 * ((total * INDEX_REPORT_FACTOR) - processed)))
             / (total * INDEX_REPORT_FACTOR))));
    fflush(stdout);

    for (int i = 0; i < NAME_INDEX_SIZE; i++) {
        fwrite(&(atomTable.nameIndex[i]), sizeof(Handle), 1, f);
    }
    printf( "Memory dump: %d%% done.\r", (int) (100 *
            (((float) processed + (0.75 * ((total * INDEX_REPORT_FACTOR) - processed)))
             / (total * INDEX_REPORT_FACTOR))));
    fflush(stdout);

    for (int i = 0; i < IMPORTANCE_INDEX_SIZE; i++) {
        fwrite(&(atomTable.importanceIndex[i]), sizeof(Handle), 1, f);
    }
    printf( "Memory dump: %d%% done.\r", (int) (100 *
            (((float) processed + (1.00 * ((total * INDEX_REPORT_FACTOR) - processed)))
             / (total * INDEX_REPORT_FACTOR))));
    fflush(stdout);

    fwrite(&atomTable.numberOfPredicateIndices, sizeof(int), 1, f);
    for (int i = 0; i < atomTable.numberOfPredicateIndices; i++) {
        fwrite(&(atomTable.predicateIndex[i]), sizeof(Handle), 1, f);
    }

    for (int i = 0; i < atomTable.numberOfPredicateIndices; i++) {
        fwrite(&(atomTable.predicateHandles[i]), sizeof(Handle), 1, f);
    }
// TODO: Save predicate type to rebuild predicateEvaluators when loading dump. For now, assuming always TreePredicateEvaluators
//    for (int i = 0; i < atomTable.numberOfPredicateIndices; i++) {
//        fwrite(&(atomTable.predicateEvaluators[i]), sizeof(Handle), 1, f);
//    }
}

void SavingLoading::load(const char *fileName, AtomSpace& atomSpace) throw (RuntimeException, IOException, InconsistenceException)
{

    clearRepositories();

    logger().fine("Starting Memory load");

    if (StatisticsMonitor::getInstance()->getAtomCount() > 0) {
        throw RuntimeException(TRACE_INFO,
                               "SavingLoading - Can only load binary image from disk into an empty atom table.");
    }
    // The above sanity check does not work if StatisticMonitor is disabled. So, makes a different check:
    if (atomSpace.getAtomTable().getSize() > 0) {
        throw RuntimeException(TRACE_INFO,
                               "SavingLoading - Can only load binary image from disk in a empty atom table.");
    }

    time_t start = time(NULL);

    // opens the file that will be read
    FILE *f = fopen(fileName, "rb");
    if (f == NULL) {
        throw IOException(TRACE_INFO,
                          "SavingLoading - Unable to open file '%s' for reading.", fileName);
    }

    // reads the file format
    char format;
    fread(&format, sizeof(char), 1, f);

    if (! (format & FULL_NETWORK_DUMP)) {
        throw RuntimeException(TRACE_INFO, "SavingLoading - invalid file format '%c'.", format);
    }

    // reads the total number of atoms. Just an idea for now.
    int atomCount = StatisticsMonitor::getInstance()->getAtomCount();
    fread(&atomCount, sizeof(int), 1, f);

    // creates a hash map from old handles to new ones
    HandleMap<Atom *> *handles = new HandleMap<Atom *>();

    processed = 0;
    total = atomCount;

    AtomTable& atomTable = atomSpace.atomTable;

    std::vector<Type> dumpToCore;
    loadClassServerInfo(f, dumpToCore);
    loadNodes(f, handles, atomTable);
    loadLinks(f, handles, atomTable);
    loadIndices(f, atomTable, handles, dumpToCore);

    // logger().info("nodes, links and indices loaded. number of handles = %d", handles->getCount());
    // update types in all atoms
    HandleMapIterator<Atom *> *it = handles->keys();
    while (it->hasNext()) {
        Atom *element = (Atom *)handles->get(it->next());
        if (dumpToCore[element->getType()] > ClassServer::getNumberOfClasses()) {
            throw InconsistenceException(TRACE_INFO,
                                         "SavingLoading - Type inconsistence clash '%d'.", element->getType());
        }
        element->type =  dumpToCore[element->getType()];
        updateHandles(element, handles);
    }
    delete(it);

    //printf( "Memory load: %d%% done.\r", (int) (100 * (((float) processed + (0.75 * ((total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    printProgress("load", (int) (100 * (((float) processed + (0.75 * ((total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    fflush(stdout);


    atomSpace.timeServer.loadRepository(f, handles);
    loadRepositories(f, handles);

    delete handles;

    fclose(f);

    // update all statistics
    StatisticsMonitor::getInstance()->reevaluateAllStatistics(atomTable);

    // calculates the total time that the process of loading has spent
    time_t duration = time(NULL) - start;
    logger().info("Memory load: 100%% done (in %d second%c).",
                  (int) duration, duration == 1 ? '\0' : 's');
    fflush(stdout);
}

void SavingLoading::loadClassServerInfo(FILE *f, std::vector<Type>& dumpToCore)
{
    logger().fine("SavingLoading::loadClassServerInfo");
    char buffer[1 << 16];
    int numTypes = ClassServer::getNumberOfClasses();

    int numTypesDump;
    fread(&numTypesDump, sizeof(int), 1, f);

    dumpToCore.resize(numTypesDump);
    for (int i = 0; i < numTypesDump; i++) {
        int classNameLength;
        fread(&classNameLength, sizeof(int), 1, f);
        fread(buffer, sizeof(char), classNameLength, f);
        buffer[classNameLength] = '\0';
        Type typeDump;
        fread(&typeDump, sizeof(Type), 1, f);

        if (!ClassServer::isDefined(buffer)) {
            dumpToCore[typeDump] = numTypes + 1;
            logger().warn("Warning: type inconsistence found (%d-%s)", typeDump, buffer);
        } else {
            dumpToCore[typeDump] = ClassServer::getType(buffer);
        }
    }

}

void SavingLoading::loadNodes(FILE *f, HandleMap<Atom *> *handles, AtomTable& atomTable)
{
    logger().fine("SavingLoading::loadNodes");

    int numNodes;
    // reads the total number of nodes
    fread(&numNodes, sizeof(int), 1, f);

    // reads each node from the file
    for (int i = 0; i < numNodes; i++) {
        Node* node = readNode(f, handles);
        node->setAtomTable(&atomTable);

        atomTable.atomSet->insert(node);

        printProgress("load", (int) (100 * ((float) ++processed / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
        fflush(stdout);
    }
    atomTable.size += numNodes;
}

void SavingLoading::loadLinks(FILE *f, HandleMap<Atom *> *handles, AtomTable& atomTable)
{
    logger().fine("SavingLoading::loadLinks");

    int numLinks;
    // reads the total number of links
    fread(&numLinks, sizeof(int), 1, f);

    // reads each link from the file
    for (int i = 0; i < numLinks; i++) {
        Link* link = readLink(f, handles);
        link->setAtomTable(&atomTable);

        atomTable.atomSet->insert(link);

        printProgress("load", (int) (100 * ((float) ++processed / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
        fflush(stdout);
    }
    atomTable.size += numLinks;
}

void SavingLoading::loadIndices(FILE *f, AtomTable& atomTable,
                                HandleMap<Atom *>* handles,
                                const std::vector<Type>& dumpToCore)
{
    logger().fine("SavingLoading::loadIndices");
    int numTypes = ClassServer::getNumberOfClasses();

    Handle* typeIndexCache = (Handle*) malloc(sizeof(Handle) * dumpToCore.size());
    Handle* targetTypeIndexCache = (Handle*) malloc(sizeof(Handle) * dumpToCore.size());

    for (int i = 0; i < numTypes; i++) {
        atomTable.typeIndex[i] = UNDEFINED_HANDLE;
        atomTable.targetTypeIndex[i] = UNDEFINED_HANDLE;
    }

    // reads the handle of each index list head from the file
    for (unsigned int i = 0; i < dumpToCore.size(); i++) {
        fread(&(typeIndexCache[i]), sizeof(Handle), 1, f);
    }
    //logger().debug("Memory load: %d%% done.\r", (int) (100 * (((float) processed + (0.25 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    printProgress("load", (int) (100 * (((float) processed + (0.25 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    fflush(stdout);
    for (unsigned int i = 0; i < dumpToCore.size(); i++) {
        fread(&(targetTypeIndexCache[i]), sizeof(Handle), 1, f);
    }

    //updating TypeIndex
    for (unsigned int i = 0; i <  dumpToCore.size(); i++) {
        atomTable.typeIndex[dumpToCore[i]] = typeIndexCache[i];
        atomTable.targetTypeIndex[dumpToCore[i]] = targetTypeIndexCache[i];
    }
    for (int i = 0; i < numTypes; i++) {
        CoreUtils::updateHandle(&(atomTable.typeIndex[i]), handles);
        CoreUtils::updateHandle(&(atomTable.targetTypeIndex[i]), handles);
    }
    //logger().debug("Memory load: %d%% done.\r", (int) (100 * (((float) processed + (0.50 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    printProgress("load", (int) (100 * (((float) processed + (0.50 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    fflush(stdout);

    for (int i = 0; i < NAME_INDEX_SIZE; i++) {
        fread(&(atomTable.nameIndex[i]), sizeof(Handle), 1, f);
    }
    for (int i = 0; i < NAME_INDEX_SIZE; i++) {
        CoreUtils::updateHandle(&(atomTable.nameIndex[i]), handles);
    }
    //logger().debug("Memory load: %d%% done.\r", (int) (100 * (((float) processed + (0.75 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    printProgress("load", (int) (100 * (((float) processed + (0.75 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    fflush(stdout);

    for (int i = 0; i < IMPORTANCE_INDEX_SIZE; i++) {
        fread(&(atomTable.importanceIndex[i]), sizeof(Handle), 1, f);
    }
    for (int i = 0; i < IMPORTANCE_INDEX_SIZE; i++) {
        CoreUtils::updateHandle(&(atomTable.importanceIndex[i]), handles);
    }
    //logger().debug("Memory load: %d%% done.\r", (int) (100 * (((float) processed + (1.00 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    printProgress("load", (int) (100 * (((float) processed + (1.00 * ((total * INDEX_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    fflush(stdout);

    fread(&atomTable.numberOfPredicateIndices, sizeof(int), 1, f);
    for (int i = 0; i < atomTable.numberOfPredicateIndices; i++) {
        fread(&(atomTable.predicateIndex[i]), sizeof(Handle), 1, f);
    }
    for (int i = 0; i < atomTable.numberOfPredicateIndices; i++) {
        fread(&(atomTable.predicateHandles[i]), sizeof(Handle), 1, f);
    }
    for (int i = 0; i < atomTable.numberOfPredicateIndices; i++) {
        CoreUtils::updateHandle(&(atomTable.predicateIndex[i]), handles);
        CoreUtils::updateHandle(&(atomTable.predicateHandles[i]), handles);
    }
    // TODO: Read predicate type to rebuild predicateEvaluators. For now, assuming always TreePredicateEvaluators
    /*    for (int i = 0; i < atomTable.numberOfPredicateIndices; i++) {
            atomTable.predicateEvaluators[i] = new TreePredicateEvaluator(atomTable.predicateHandles[i]);
            atomTable.predicateHandles2Indices->add(atomTable.predicateHandles[i],(int*)i);
        }*/

    free(typeIndexCache);
    free(targetTypeIndexCache);
}

void SavingLoading::updateHandles(Atom *atom, HandleMap<Atom *> *handles)
{
    logger().fine("SavingLoading::updateHandles: atom = %p, type = %d", atom, atom->getType());

    // if atom uses a CompositeTruthValue, updates the version handles inside it
    if (atom->getTruthValue().getType() == COMPOSITE_TRUTH_VALUE) {
        //logger().fine("SavingLoading::updateHandles: CTV");
        CompositeTruthValue ctv((const CompositeTruthValue&) atom->getTruthValue());
        ctv.updateVersionHandles(handles);
        atom->setTruthValue(ctv);
    }

    //logger().fine("SavingLoading::updateHandles: incoming set");
    // updates the handles for the atom's incoming set
    for (HandleEntry *p = atom->incoming; p != NULL; p = p->next) {
        CoreUtils::updateHandle(&p->handle, handles);
    }

    Link *link = dynamic_cast<Link *>(atom);
    if (link && (link->getArity() > 0)) {
        //logger().fine("SavingLoading::updateHandles: outgoing set");
        AtomTable& atomTable = *(atom->getAtomTable());

        // BUG FIX: Links should be updated in AtomSet because it still uses old handles in its outgoing
        atomTable.atomSet->erase(atom);
//        printf("AtomTable[%d]::atomSet->erase(%p) => size = %d\n", t, atom, atomTable.atomSet->size());
        // updates the handles for the atom outgoing set
        for (int i = 0; i < link->getArity(); i++) {
            CoreUtils::updateHandle(&(link->outgoing[i]), handles);
        }

        if (ClassServer::isAssignableFrom(UNORDERED_LINK, atom->type)) {
#ifdef USE_STD_VECTOR_FOR_OUTGOING
            std::sort(link->outgoing.begin(), link->outgoing.end(), CoreUtils::HandleComparison());
#else
            qsort(link->outgoing, link->getArity(), sizeof(Handle), CoreUtils::handleCompare);
#endif
        }

        atomTable.atomSet->insert(atom);
//        printf("AtomTable[%d]::atomSet->insert(%p) => size = %d\n", t, atom, atomTable.atomSet->size());
    }

    //logger().fine("SavingLoading::updateHandles: indices");
    // updates the handles for indices
    for (int i = 0; i < NUMBER_OF_INDICES; i++) {
        CoreUtils::updateHandle(&(atom->indices[i]), handles);
    }
    //logger().fine("SavingLoading::updateHandles: targetTypeIndices");
    int targetTypeSize = atom->getTargetTypeIndexSize();
    for (int i = 0; i < targetTypeSize; i++) {
        CoreUtils::updateHandle(&(atom->targetTypeIndex[i]), handles);
    }
    if (atom->predicateIndexInfo) {
        //logger().fine("SavingLoading::updateHandles: predicateIndices");
        int size = bitcount(atom->predicateIndexInfo->predicateIndexMask);
        for (int i = 0; i < size; i++) {
            CoreUtils::updateHandle(&(atom->predicateIndexInfo->predicateIndex[i]), handles);
        }
    }

    // updates handles for trail
    if (ClassServer::isAssignableFrom(LINK, atom->type)) {
        Trail *t = ((Link *)atom)->getTrail();
        if (t->getSize()) {
            //logger().fine("SavingLoading::updateHandles: trails");
            Trail *newTrail = new Trail();
            for (int i = 0; i < t->getSize(); i++) {
                Handle handle = t->getElement(i);
                CoreUtils::updateHandle(&handle, handles);
                newTrail->insert(handle);
            }
            ((Link *)atom)->setTrail(newTrail);
            //newTrail->print();
            delete(t);
        }
    }
}

void SavingLoading::writeAtom(FILE *f, Atom *atom)
{
    logger().fine("SavingLoading::writeAtom: %p (type = %d)", atom, atom->getType());

    // writes the atom type
    fwrite(&atom->type, sizeof(Type), 1, f);
    // writes the atom flags
    fwrite(&atom->flags, sizeof(char), 1, f);

    // writes the atom handle
    Handle handle = TLB::getHandle(atom);
    fwrite(&handle, sizeof(Handle), 1, f);

    // store the position on the file where the number of incoming atoms will be
    // written.
    int incomingOffset = ftell(f);
    int incomingSize = 0;
    fwrite(&incomingSize, sizeof(int), 1, f);

    // for each incoming atom, it will be written on the file and the number
    // of incoming atoms will be incremented
    for (HandleEntry *p = atom->incoming; p != NULL; p = p->next) {
        fwrite(&p->handle, sizeof(Handle), 1, f);
        incomingSize++;
    }

    // rewind to position where incomming set size must be written
    fseek(f, incomingOffset, SEEK_SET);

    // writes incomming set size in the proper position
    fwrite(&incomingSize, sizeof(int), 1, f);

    // returns the pointer back to the end of the file
    fseek(f, 0, SEEK_END);

    // writes the indices of the atom
    fwrite(atom->indices, sizeof(Handle), NUMBER_OF_INDICES, f);

    // writes the predicate indices of the atom
    bool hasPredicateIndices = (atom->predicateIndexInfo != NULL);
    fwrite(&hasPredicateIndices, sizeof(bool), 1, f);
    if (hasPredicateIndices) {
        unsigned long mask = atom->predicateIndexInfo->predicateIndexMask;
        fwrite(&mask, sizeof(unsigned long), 1, f);
        int size = bitcount(mask);
        fwrite(&size, sizeof(int), 1, f);
        fwrite(atom->predicateIndexInfo->predicateIndex, sizeof(Handle), size, f);
    }

    // writes the Attention Value
    writeAttentionValue(f, atom->getAttentionValue());

    // writes the Truth Value
    writeTruthValue(f, atom->getTruthValue());
}

void SavingLoading::readAtom(FILE *f, HandleMap<Atom *> *handles, Atom *atom)
{
    logger().fine("SavingLoading::readAtom()");

    // reads the atom type
    //atom->type
    fread(&atom->type, sizeof(Type), 1, f);
//    if((int)type > ClassServer::getNumberOfClasses()){
//        fprintf(stdout, "Undefined type read %d\n", type);
//        fflush(stdout);
//    }

    // reads the atom flags
    fread(&atom->flags, sizeof(char), 1, f);

    // reads the atom handle
    Handle atomHandle;
    fread(&atomHandle, sizeof(Handle), 1, f);

    if (handles != NULL) {
        handles->add(atomHandle, atom);
        logger().fine("Added handles in map: %p => %p (type = %d)", atomHandle, atom, atom->getType());
    } else {
        logger().warn("No HandleMap while reading atom from file: %p (type = %d)", atom, atom->getType());
    }

    // reads the incoming set of the atom, including the number of incoming
    // atoms
    int incomingSize;
    fread(&incomingSize, sizeof(int), 1, f);
    for (int i = 0; i < incomingSize; i++) {
        Handle incomingHandle;
        fread(&incomingHandle, sizeof(Handle), 1, f);
        atom->addIncomingHandle(incomingHandle);
    }

    // reads the atom's indices
    for (int i = 0; i < NUMBER_OF_INDICES; i++) {
        fread(&(atom->indices[i]), sizeof(Handle), 1, f);
    }

    // reads the predicate indices of the atom
    bool hasPredicateIndices;
    fread(&hasPredicateIndices, sizeof(bool), 1, f);
    if (hasPredicateIndices) {
        atom->predicateIndexInfo = new PredicateIndexStruct();
        unsigned long mask;
        fread(&mask, sizeof(unsigned long), 1, f);
        atom->predicateIndexInfo->predicateIndexMask = mask;
        int size;
        fread(&size, sizeof(int), 1, f);
        atom->predicateIndexInfo->predicateIndex = new Handle[size];
        for (int i = 0; i < size; i++) {
            fread(&(atom->predicateIndexInfo->predicateIndex[i]), sizeof(Handle), 1, f);
        }
    }

    // reads AttentionValue
    AttentionValue *av = readAttentionValue(f);
    atom->setAttentionValue(*av);
    delete (av);

    TruthValue *tv = readTruthValue(f);
    atom->setTruthValue(*tv);
    delete (tv);

}

void SavingLoading::writeNode(FILE *f, Node *node)
{

    writeAtom(f, node);

    // writes the node's name on the file
    int nameSize = node->name.length();
    fwrite(&nameSize, sizeof(int), 1, f);
    if (nameSize > 0) {
        fwrite(node->name.c_str(), sizeof(char), nameSize, f);
    }
}

Node* SavingLoading::readNode(FILE *f, HandleMap<Atom *> *handles)
{
    logger().fine("SavingLoading::readNode()");
    // a new node is created
    Node *node = new Node(NODE, "");

    // the atom properties of the node is read from the file
    readAtom(f, handles, node);

    // the node's name is read from the file
    int nameSize;
    fread(&nameSize, sizeof(int), 1, f);
    if (nameSize > 0) {
        char *name = new char[nameSize+1];
        fread(name, sizeof(char), nameSize, f);
        name[nameSize] = '\0';
        node->name = name;
        delete[](name);
    }

    return node;
}

void SavingLoading::writeLink(FILE *f, Link *link)
{
    logger().fine("writeLink(): %s", link->toString().c_str());

    //link->getTrail()->print();

    writeAtom(f, link);

    // the link's arity is written on the file
    Arity arity = link->getArity();
    fwrite(&arity, sizeof(Arity), 1, f);

    // the link's outgoing set is written on the file
    for (int i = 0; i < arity; i++) {
        //logger().fine("writeLink(): outgoing[%d] => %p: %s", i, link->outgoing[i], link->outgoing[i]->toString().c_str());
        fwrite(&(link->outgoing[i]), sizeof(Handle), 1, f);
    }

    // the target type indices of the link is written on the file
    int targetTypeSize = link->getTargetTypeIndexSize();
    fwrite(&targetTypeSize, sizeof(int), 1, f);
    fwrite(link->targetTypeIndex, sizeof(Handle), targetTypeSize, f);

    // the trail
    Trail *trail = link->getTrail();

    int trailSize = trail->getSize();
    fwrite(&trailSize, sizeof(int), 1, f);
    for (int i = 0; i < trailSize; i++) {
        Handle handle = trail->getElement(i);
        fwrite(&handle, sizeof(Handle), 1, f);
    }

}

void SavingLoading::writeAttentionValue(FILE *f, const AttentionValue& attentionValue)
{
    AttentionValue::sti_t tempSTI = attentionValue.getSTI();
    AttentionValue::lti_t tempLTI = attentionValue.getLTI();
    AttentionValue::vlti_t tempVLTI = attentionValue.getVLTI();

    fwrite(&tempSTI, sizeof(AttentionValue::sti_t), 1, f);
    fwrite(&tempLTI, sizeof(AttentionValue::lti_t), 1, f);
    fwrite(&tempVLTI, sizeof(AttentionValue::vlti_t), 1, f);
}

void SavingLoading::writeTruthValue(FILE *f, const TruthValue& tv)
{
    string tvStr = tv.toString();
    TruthValueType type = tv.getType();
    int length = tvStr.size();

    fwrite(&type, sizeof(TruthValueType), 1, f);
    fwrite(&length, sizeof(int), 1, f);
    fwrite(tvStr.c_str(), sizeof(char), length, f);
}

AttentionValue *SavingLoading::readAttentionValue(FILE *f)
{
    AttentionValue::sti_t tempSTI;
    AttentionValue::lti_t tempLTI;
    AttentionValue::vlti_t tempVLTI;

    fread(&tempSTI, sizeof(AttentionValue::sti_t), 1, f);
    fread(&tempLTI, sizeof(AttentionValue::lti_t), 1, f);
    fread(&tempVLTI, sizeof(AttentionValue::vlti_t), 1, f);

    return(AttentionValue::factory(tempSTI, tempLTI, tempVLTI));
}


TruthValue *SavingLoading::readTruthValue(FILE *f)
{
    //logger().fine("SavingLoading::readTruthValue()");
    TruthValueType type;
    int length;

    fread(&type, sizeof(TruthValueType), 1, f);
    fread(&length, sizeof(int), 1, f);
    //logger().fine("SavingLoading::readTruthValue() type = %d, length =  %d", type, length);
    char *tvStr = new char[length+1];
    fread(tvStr, sizeof(char), length, f);
    tvStr[length] = '\0';

    //logger().fine("SavingLoading::readTruthValue() tvStr = %s\n", tvStr);
    TruthValue* result = TruthValue::factory(type, tvStr);
    delete[] tvStr;
    return result;
}

Link *SavingLoading::readLink(FILE *f, HandleMap<Atom *> *handles)
{
    // a new link is created
    Link *link = new Link(LINK, std::vector<Handle>());

    readAtom(f, handles, link);

#ifdef USE_STD_VECTOR_FOR_OUTGOING
    // the link's arity is read from the file
    Arity arity;
    fread(&arity, sizeof(Arity), 1, f);

    // the link's outgoing set is read from the file
    for (int i = 0; i < arity; i++) {
        Handle h;
        fread(&h, sizeof(Handle), 1, f);
        link->outgoing.push_back(h);
    }
#else
    // the link's arity is read from the file
    fread(&link->arity, sizeof(Arity), 1, f);

    // the link's outgoing set is read from the file
    link->outgoing = (Handle*) malloc(sizeof(Handle) * link->arity);
    for (int i = 0; i < link->arity; i++) {
        fread(&(link->outgoing[i]), sizeof(Handle), 1, f);
    }
#endif

    // the target type indices is read from the file
    int targetTypeSize;
    fread(&targetTypeSize, sizeof(int), 1, f);
    link->targetTypeIndex = new Handle[targetTypeSize];
    for (int i = 0; i < targetTypeSize; i++) {
        fread(&(link->targetTypeIndex[i]), sizeof(Handle), 1, f);
    }

    // the trail
    Trail *trail = link->getTrail();
    int trailSize;
    fread(&trailSize, sizeof(int), 1, f);
    for (int i = 0; i < trailSize; i++) {
        Handle handle;
        fread(&handle, sizeof(Handle), 1, f);
        trail->insert(handle, false);
    }

    return link;
}

void SavingLoading::printProgress(const char *s, int n)
{
    static int old = -1;

    if (old != n) {
        old = n;
        logger().debug("Memory %s: %d%% done.\r", s, n);
    }
}

void SavingLoading::addSavableRepository(SavableRepository *repository) throw (RuntimeException)
{
    const char* id = repository->getId();

    RepositoryHash::const_iterator it = repositories.find(id);
    if (it != repositories.end()) {
        throw RuntimeException(TRACE_INFO, "SavingLoading - Duplicated repository ids: '%s'.", id);
    }

    repositories[id] = repository;
}

void SavingLoading::saveRepositories(FILE *f)
{
    logger().fine("SavingLoading::saveRepositories");
    unsigned int size = repositories.size();
    fwrite(&size, sizeof(unsigned int), 1, f);

    for (RepositoryHash::const_iterator it = repositories.begin(); it != repositories.end(); it++) {
        const std::string& repId = it->first;
        int idSize = repId.length() + 1;
        fwrite(&idSize, sizeof(int), 1, f);
        fwrite(repId.c_str(), sizeof(char), idSize, f);

        logger().debug("Saving repository: %s", repId.c_str());
        SavableRepository* rep = it->second;
        rep->saveRepository(f);

    }
}

void SavingLoading::loadRepositories(FILE *f, HandleMap<Atom *> *conv) throw (RuntimeException)
{
    logger().fine("SavingLoading::loadRepositories");
    unsigned int size;
    fread(&size, sizeof(unsigned int), 1, f);

    if (size != repositories.size()) {
        logger().warn("Number of repositories in dump file (%d) is different from number of registered repositories (%d)", size, repositories.size());
        return;
    }

    for (unsigned int i = 0; i < size; i++) {
        int idSize;
        fread(&idSize, sizeof(int), 1, f);

        auto_ptr<char> id(new char[idSize]);

        fread(id.get(), sizeof(char), idSize, f);

        logger().debug("Loading repository: %s\n", id.get());

        RepositoryHash::const_iterator it = repositories.find((const char *)id.get());

        if (it == repositories.end()) {
            throw RuntimeException(TRACE_INFO,
                                   "SavingLoading - Unknown repository id: '%s'.", id.get());
        }


        it->second->loadRepository(f, conv);
    }

}

void SavingLoading::clearRepositories()
{
    logger().fine("SavingLoading::clearRepositories");
    for (RepositoryHash::const_iterator it = repositories.begin();
            it != repositories.end(); it++) {

        it->second->clear();
    }
}
