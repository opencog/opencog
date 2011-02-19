/*
 * opencog/persist/file/SavingLoading.cc
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

#include "CompositeRenumber.h"
#include "SavingLoading.h"
#include "SpaceServerSavable.h"
#include "TimeServerSavable.h"
#include "CoreUtils.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <opencog/util/platform.h>

#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CompositeTruthValue.h>
#include <opencog/atomspace/HandleIterator.h>
#include <opencog/atomspace/HandleMap.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/StatisticsMonitor.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>

using namespace opencog;

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

    // TODO: bad bad - saving and loading should be integrated as a request or
    // use the AtomSpace API.
    AtomTable& atomTable = const_cast<AtomTable&>(atomSpace.atomSpaceAsync->getAtomTable());

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

    TimeServerSavable tss;
    tss.setServer(&atomSpace.atomSpaceAsync->getTimeServer());
    tss.saveRepository(f);

    SpaceServerSavable sss;
    sss.setServer(&atomSpace.atomSpaceAsync->getSpaceServer());
    sss.saveRepository(f);

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
    int numTypes = classserver().getNumberOfClasses();

    fwrite(&numTypes, sizeof(int), 1, f);

    for (Type i = 0; i < numTypes; i++) {
        int classNameLength = classserver().getTypeName(i).length();
        fwrite(&classNameLength, sizeof(int), 1, f);
        fwrite(classserver().getTypeName(i).c_str(), sizeof(char), classNameLength, f);
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
        Node* node = dynamic_cast<Node*>(TLB::getAtom(atomHandle));
        if ( node == NULL ) {
            logger().error( "Trying to save a node which isn't registered at TLB. Handle %d", atomHandle.value() );
            continue;
        } // if
        logger().info( "Saving Node handle %d name %s", atomHandle.value(), node->toString().c_str() );
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
    atomCount -= numNodes;

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

    /**
     * All handles must be copied to a single set to keep the ordering
     * of all handle types. Ex. There are more than one types of links.
     * Each type has it's instances keeped at a distinct set of handles.
     * An iterator travels each set of each type. So, it will not keep
     * the increasing ordering in a situation like:
     * Suppose that a ListLink is inserted with handle.value() = 65558
     * Then, an ExecutionLink is inserted with handle.value() = 65559
     * Finally, a ListLink is inserted with handle.value() = 65560
     * The first and second links are outgoing of the third link.
     *
     * If we use a simple HandleIterator, the links will be retrieved
     * at the following sequence: 65558, 65560, 65559. This way, the links
     * will be saved in a non increasing sequence and when the loadLinks
     * was called, a segmentation fault will occour given that link
     * 65560 requires 65559 but the the last one wasn't loaded yet.
     */
    std::set<Handle> linkHandles;
    while (iter->hasNext()) {
        linkHandles.insert( iter->next( ) );
    } // while
    delete iter;

    // writes links to file and increments link counter
    std::set<Handle>::iterator itLinks;
    for( itLinks = linkHandles.begin( ); itLinks != linkHandles.end( ); ++itLinks ) {
        Link* link = dynamic_cast<Link*>(TLB::getAtom(*itLinks));
        logger().fine( "Saving Link handle %d name %s", itLinks->value(), link->toString().c_str() );
        writeLink(f, link);
        numLinks++;
        printf( "Memory dump: %d%% done.\r", (int) (100 * ((float) ++processed / (total * INDEX_REPORT_FACTOR))));
        fflush(stdout);        
    } // for

    // rewind to position where number of links must be written
    fseek(f, numLinksOffset, SEEK_SET);

    // writes the number of links in the proper position
    fwrite(&numLinks, sizeof(int), 1, f);

    // updates the atomCount
    atomCount -= numLinks;

    // returns the pointer back to the end of the file
    fseek(f, 0, SEEK_END);
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
    if (atomSpace.getSize() > 0) {
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

    AtomTable& atomTable = const_cast<AtomTable&>(atomSpace.atomSpaceAsync->getAtomTable());

    std::vector<Type> dumpToCore;
    loadClassServerInfo(f, dumpToCore);
    loadNodes(f, handles, atomTable, dumpToCore);
    loadLinks(f, handles, atomTable, dumpToCore);

    HandleMapIterator<Atom *> *it = handles->keys();
    while (it->hasNext()) {
        Atom *element = (Atom *)handles->get(it->next());
        updateHandles(element, handles);
    }
    delete(it);

    printProgress("load", (int) (100 * (((float) processed + (0.75 * ((total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
    fflush(stdout);


    TimeServerSavable tss;
    tss.setServer(&atomSpace.atomSpaceAsync->getTimeServer());
    tss.loadRepository(f, handles);

    SpaceServerSavable sss;
    sss.setServer(&atomSpace.atomSpaceAsync->getSpaceServer());
    sss.loadRepository(f, handles);

    loadRepositories(f, handles);

    delete handles;

    fclose(f);

    // update all statistics
    StatisticsMonitor::getInstance()->reevaluateAllStatistics(atomSpace);

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
    int numTypes = classserver().getNumberOfClasses();

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

        if (!classserver().isDefined(buffer)) {
            dumpToCore[typeDump] = numTypes + 1;
            logger().warn("Warning: type inconsistence found (%d-%s)", typeDump, buffer);
        } else {
            dumpToCore[typeDump] = classserver().getType(buffer);
        }
    }

}

void SavingLoading::loadNodes(FILE *f, HandleMap<Atom *> *handles, AtomTable& atomTable, const std::vector<Type>& dumpToCore )
{
    logger().fine("SavingLoading::loadNodes");

    int numNodes;
    // reads the total number of nodes
    fread(&numNodes, sizeof(int), 1, f);    

    // reads each node from the file
    for (int i = 0; i < numNodes; i++) {
        Node *node = new Node(NODE, "");
        
        Type oldType;
        fread(&oldType, sizeof(Type), 1, f);        
        if (dumpToCore[oldType] > classserver().getNumberOfClasses()) {
            throw InconsistenceException(TRACE_INFO,
                                         "SavingLoading - Type inconsistence clash '%d'.", oldType );
        }
        node->type = dumpToCore[oldType];
        readNode(f, node, handles);

        atomTable.add( node );
        printProgress("load", (int) (100 * ((float) ++processed / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
        fflush(stdout);
    }

}

void SavingLoading::loadLinks(FILE *f, HandleMap<Atom *> *handles, AtomTable& atomTable, const std::vector<Type>& dumpToCore)
{
    logger().fine("SavingLoading::loadLinks");

    int numLinks;
    // reads the total number of links
    fread(&numLinks, sizeof(int), 1, f);

    /**
     * Before registering indexes for a loaded link, all it's outgoing handles must be
     * already loaded in order to avoid INVALID_HANDLE issues. So, a handle list
     * will be used to store all the link handles registered at TLB. Then, all indexes 
     * related to each link will be created.
     */
    // first reads each link from the file
    for (int i = 0; i < numLinks; i++) {
        // a new link is created
        Link *link = new Link(LINK, std::vector<Handle>());

        Type oldType;
        fread(&oldType, sizeof(Type), 1, f);        
        if (dumpToCore[oldType] > classserver().getNumberOfClasses()) {
            throw InconsistenceException(TRACE_INFO,
                                         "SavingLoading - Type inconsistence clash '%d'.", oldType );
        }
        link->type = dumpToCore[oldType];
        readLink( f, link, handles );

        atomTable.add(link);
    } // for
}

void SavingLoading::updateHandles(Atom *atom, HandleMap<Atom *> *handles)
{
    logger().fine("SavingLoading::updateHandles: atom = %p, type = %d", atom, atom->getType());

    // if atom uses a CompositeTruthValue, updates the version handles inside it
    if (atom->getTruthValue().getType() == COMPOSITE_TRUTH_VALUE) {
        //logger().fine("SavingLoading::updateHandles: CTV");
        CompositeTruthValue ctv((const CompositeTruthValue&) atom->getTruthValue());
        CompositeRenumber::updateVersionHandles(ctv, handles);
        atom->setTruthValue(ctv);        
    } // if
    
    // updates handles for trail
    if (classserver().isA(atom->type, LINK)) {
        Trail *t = ((Link *)atom)->getTrail();
        if (t->getSize()) {
            //logger().fine("SavingLoading::updateHandles: trails");
            Trail *newTrail = new Trail();
            for (size_t i = 0; i < t->getSize(); i++) {
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
    logger().info("SavingLoading::writeAtom: %p (type = %d) (handle = %d)", atom, atom->getType(), atom->getHandle().value());

    // writes the atom type
    fwrite(&atom->type, sizeof(Type), 1, f);
    // writes the atom flags
    fwrite(&atom->flags, sizeof(char), 1, f);

    // writes the atom handle
    Handle handle = atom->getHandle();
    fwrite(&handle, sizeof(Handle), 1, f);

    // incoming references will be re-created during the loading process

    // writes the Attention Value
    writeAttentionValue(f, atom->getAttentionValue());

    // writes the Truth Value
    writeTruthValue(f, atom->getTruthValue());
}

void SavingLoading::readAtom(FILE *f, HandleMap<Atom *> *handles, Atom *atom)
{
    logger().fine("SavingLoading::readAtom()");


    // reads the atom flags
    fread(&atom->flags, sizeof(char), 1, f);

    // reads the atom handle
    Handle atomHandle;
    fread(&atomHandle, sizeof(Handle), 1, f);

    if (handles != NULL) {
        handles->add(atomHandle, atom);
        logger().fine("Added handles in map: %p => %p (type = %d)", atomHandle.value(), atom, atom->getType());
    } else {
        logger().warn("No HandleMap while reading atom from file: %p (type = %d)", atom, atom->getType());
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

void SavingLoading::readNode(FILE *f, Node* node, HandleMap<Atom *> *handles) 
{
    logger().fine("SavingLoading::readNode()");

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
    std::string tvStr = tv.toString();
    logger().info( "SavingLoading::writeTruthValue() tvStr = %s\n", tvStr.c_str());
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
    logger().info("SavingLoading::readTruthValue() tvStr = %s\n", tvStr);
    TruthValue* result = TruthValue::factory(type, tvStr);
    delete[] tvStr;
    return result;
}

void SavingLoading::readLink(FILE *f, Link *link, HandleMap<Atom *> *handles)
{
    logger().fine("SavingLoading::readLink()");
    readAtom(f, handles, link);

    // the link's arity is read from the file
    Arity arity;
    fread(&arity, sizeof(Arity), 1, f);

    // the link's outgoing set is read from the file
    for (int i = 0; i < arity; i++) {
        Handle h;
        fread(&h, sizeof(Handle), 1, f);
        link->outgoing.push_back( handles->get(h)->getHandle() );
    }

    // the trail
    Trail *trail = link->getTrail();
    int trailSize;
    fread(&trailSize, sizeof(int), 1, f);
    for (int i = 0; i < trailSize; i++) {
        Handle handle;
        fread(&handle, sizeof(Handle), 1, f);
        trail->insert( handle, false);
    }
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

        std::auto_ptr<char> id(new char[idSize]);

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
