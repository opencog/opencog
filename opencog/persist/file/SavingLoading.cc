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

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/HandleMap.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>
#include <opencog/util/macros.h>
#include <opencog/util/functional.h>
#include <opencog/util/platform.h>

#include "SavingLoading.h"
#include "SpaceServerSavable.h"
#include "TimeServerSavable.h"
#include "CoreUtils.h"

using namespace opencog;

#define FULL_NETWORK_DUMP          (1 << 0)

#define INDEX_REPORT_FACTOR             1.02
#define POST_PROCESSING_REPORT_FACTOR   1.10

int processed = 0;
int total = 0;

SavingLoading::SavingLoading()
{
}

void SavingLoading::save(const char *fileName,
                         AtomSpace& atomSpace,
                         SpaceServer& spacs,
                         TimeServer& tims)
    throw (IOException)
{
    logger().info("Saving OpenCog instance");

    time_t start = time(NULL);

    // opens the file that will be modified
    FILE *f = fopen(fileName, "wb");
    if (f == NULL) {
        throw IOException(TRACE_INFO,
                          "SavingLoading - Unable to open file '%s' for writing", fileName);
    }

    // TODO: bad bad - saving and loading should be integrated as a request or
    // use the AtomSpace API.
    AtomTable& atomTable = const_cast<AtomTable&> (atomSpace.getAtomTable());

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
    tss.setServer(&tims);
    tss.saveRepository(f);

    SpaceServerSavable sss;
    sss.setServer(&spacs);
    sss.saveRepository(f);

    saveRepositories(f);

    // closes the file
    fclose(f);

    // calculates the total time that the process of saving has spent
    time_t duration = time(NULL) - start;
    printf( "Memory dump: 100%% done.\n");
    fflush(stdout);
    logger().info("Memory dump complete in %d second%c.",
                  (int) duration, duration == 1 ? '\0' : 's');
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

    // writes nodes to file and increments node counter
    atomTable.foreachHandleByType(
        [&](Handle atomHandle)->void
        {
            NodePtr node(NodeCast(atomHandle));
            if ( node == NULL ) {
                logger().error( "Trying to save a node which isn't in atomTable "
                        "(%p). Handle %d", &atomTable, atomHandle.value() );
                return;
            }
            logger().fine( "Saving Node handle %d name %s", atomHandle.value(),
                    node->toString().c_str() );
            writeNode(f, node);
            numNodes++;
            int percentage = (int) (100 * ((float) ++processed /
                        (total * INDEX_REPORT_FACTOR)));
            if ((percentage % 10) == 0) {
                printf( "Memory dump: %d%% done.\r", percentage);
                fflush(stdout);
            }
        },
        NODE, true);


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
     * If we use a simple Handle iterator, the links will be retrieved
     * at the following sequence: 65558, 65560, 65559. This way, the links
     * will be saved in a non increasing sequence and when the loadLinks
     * was called, a segmentation fault will occour given that link
     * 65560 requires 65559 but the the last one wasn't loaded yet.
     */
    std::set<Handle> linkHandles;
    atomTable.getHandlesByType(inserter(linkHandles), LINK, true);

    // writes links to file and increments link counter
    std::set<Handle>::iterator itLinks;
    for (itLinks = linkHandles.begin( ); itLinks != linkHandles.end( );
            ++itLinks ) {
        LinkPtr link(LinkCast(*itLinks));
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

void SavingLoading::load(const char *fileName,
                         AtomSpace& atomSpace,
                         SpaceServer& spacs,
                         TimeServer& tims)
    throw (RuntimeException, IOException, InconsistenceException)
{
    clearRepositories();

    logger().fine("Starting Memory load");

    // sanity check
    if (atomSpace.getSize() > 0) {
        throw RuntimeException(TRACE_INFO,
            "SavingLoading - Can only load binary image from disk into an empty atom table.");
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
    size_t rc = fread(&format, sizeof(char), 1, f);

    if (! (rc == 1 && format & FULL_NETWORK_DUMP)) {
        throw RuntimeException(TRACE_INFO, "SavingLoading - invalid file format '%c'.", format);
    }

    // reads the total number of atoms. Just an idea for now.
    int atomCount = 0;
    if ( fread(&atomCount, sizeof(int), 1, f) != 1 ) {
        throw RuntimeException(TRACE_INFO, "SavingLoading - failed to read.");
    }

    // creates a hash map from old handles to new ones
    HandMapPtr handles = std::make_shared<HandleMap<AtomPtr>>();

    processed = 0;
    total = atomCount;

    AtomTable& atomTable = const_cast<AtomTable&>(atomSpace.getAtomTable());

    std::vector<Type> dumpToCore;
    loadClassServerInfo(f, dumpToCore);
    loadNodes(f, handles, atomTable, dumpToCore);
    loadLinks(f, handles, atomTable, dumpToCore);

    HandleMapIterator<AtomPtr> *it = handles->keys();
    while (it->hasNext()) {
        AtomPtr element(handles->get(it->next()));
        updateHandles(element, handles);
    }
    delete(it);

    printProgress("load", (int) (100 * (((float) processed + (0.75 * ((total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR) - processed))) / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));

    TimeServerSavable tss;
    tss.setServer(&tims);
    tss.loadRepository(f, handles);

    SpaceServerSavable sss;
    sss.setServer(&spacs);
    sss.loadRepository(f, handles);

    loadRepositories(f, handles);

    fclose(f);

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
    bool b_read = true;
    FREAD_CK(&numTypesDump, sizeof(int), 1, f);

    dumpToCore.resize(numTypesDump);
    for (int i = 0; i < numTypesDump; i++) {
        int classNameLength;
        FREAD_CK(&classNameLength, sizeof(int), 1, f);
        FREAD_CK(buffer, sizeof(char), classNameLength, f);
        buffer[classNameLength] = '\0';
        Type typeDump;
        FREAD_CK(&typeDump, sizeof(Type), 1, f);

        if (!classserver().isDefined(buffer)) {
            dumpToCore[typeDump] = numTypes + 1;
            logger().warn("Warning: type inconsistence found (%d-%s)", typeDump, buffer);
        } else {
            dumpToCore[typeDump] = classserver().getType(buffer);
        }
    }
    CHECK_FREAD;
}

void SavingLoading::loadNodes(FILE *f, HandMapPtr handles, AtomTable& atomTable, const std::vector<Type>& dumpToCore )
{
    logger().fine("SavingLoading::loadNodes");

    int numNodes;
    bool b_read = true;
    // reads the total number of nodes
    FREAD_CK(&numNodes, sizeof(int), 1, f);

    // reads each node from the file
    for (int i = 0; i < numNodes; i++) {

        Type oldType;
        size_t rc = FREAD_CK(&oldType, sizeof(Type), 1, f);
        if (rc < 0)
            throw InconsistenceException(TRACE_INFO,
                                         "SavingLoading - Failed read.");
        if (dumpToCore[oldType] > classserver().getNumberOfClasses()) {
            throw InconsistenceException(TRACE_INFO,
                                         "SavingLoading - Type inconsistence clash '%d'.", oldType );
        }
        Type newtype = dumpToCore[oldType];
        NodePtr node(readNode(f, newtype, handles));

        atomTable.add( node );
        printProgress("load", (int) (100 * ((float) ++processed / (total * INDEX_REPORT_FACTOR * POST_PROCESSING_REPORT_FACTOR))));
        fflush(stdout);
    }
    CHECK_FREAD;
}

void SavingLoading::loadLinks(FILE* f, HandMapPtr handles,
        AtomTable& atomTable, const std::vector<Type>& dumpToCore)
{
    logger().fine("SavingLoading::loadLinks");

    int numLinks;
    bool b_read = true;
    // reads the total number of links
    FREAD_CK(&numLinks, sizeof(int), 1, f);

    // Before registering indexes for a loaded link, all of its outgoing handles
    // must be already loaded in order to avoid INVALID_HANDLE issues. So,
    // a handle list will be used to store all the link handles registered
    // with the AtomTable. Then, all indexes related to each link will be created.
    //
    // first reads each link from the file
    for (int i = 0; i < numLinks; i++) {
        // a new link is created

        Type oldType;
        FREAD_CK(&oldType, sizeof(Type), 1, f);
        if (dumpToCore[oldType] > classserver().getNumberOfClasses()) {
            throw InconsistenceException(TRACE_INFO,
                 "SavingLoading - Type inconsistence clash '%d'.", oldType );
        }
        Type t = dumpToCore[oldType];
        LinkPtr link(readLink(f, t, handles));
        atomTable.add(link);
    }
    CHECK_FREAD;
}

void SavingLoading::updateHandles(AtomPtr atom, HandMapPtr handles) {}

void SavingLoading::writeAtom(FILE *f, AtomPtr atom)
{
    logger().fine("SavingLoading::writeAtom: (type = %d) (handle = %d)", atom->getType(), atom->getHandle().value());

    // writes the atom type
    Type type = atom->getType();
    fwrite(&type, sizeof(Type), 1, f);
    // writes the atom flags
    char flags = atom->_flags;
    fwrite(&flags, sizeof(char), 1, f);

    // writes the atom handle
    UUID uuid = atom->getHandle().value();
    fwrite(&uuid, sizeof(UUID), 1, f);

    // incoming references will be re-created during the loading process

    // writes the Attention Value
    writeAttentionValue(f, atom->getAttentionValue());

    // writes the Truth Value
    writeTruthValue(f, atom->getTruthValue());
}

Handle SavingLoading::readAtom(FILE *f, AtomPtr atom)
{
    logger().fine("SavingLoading::readAtom()");

    // reads the atom flags
    char flags;
    bool b_read = true;
    FREAD_CK(&flags, sizeof(char), 1, f);
    atom->_flags = flags;

    // reads the atom handle
    UUID uuid;
    FREAD_CK(&uuid, sizeof(UUID), 1, f);
    atom->_uuid = uuid;

    // reads AttentionValue
    AttentionValuePtr av = readAttentionValue(f);
    atom->setAttentionValue(av);

    TruthValuePtr tv = readTruthValue(f);
    atom->setTruthValue(tv);

    CHECK_FREAD;

    return Handle(atom);
}

void SavingLoading::writeNode(FILE *f, NodePtr node)
{

    writeAtom(f, node);

    // writes the node's name on the file
    int nameSize = node->getName().length();
    fwrite(&nameSize, sizeof(int), 1, f);
    if (nameSize > 0) {
        fwrite(node->getName().c_str(), sizeof(char), nameSize, f);
    }
}

NodePtr SavingLoading::readNode(FILE* f, Type t, HandMapPtr handles)
{
    logger().fine("SavingLoading::readNode()");

    // the atom properties of the node is read from the file
    AtomPtr junk(createNode(t, ""));
    Handle h = readAtom(f, junk);

    // the node's name is read from the file
    std::string nam;
    int nameSize;
    bool b_read = true;
    FREAD_CK(&nameSize, sizeof(int), 1, f);
    if (nameSize > 0) {
        char *name = new char[nameSize+1];
        FREAD_CK(name, sizeof(char), nameSize, f);
        name[nameSize] = '\0';
        nam = name;
        delete[](name);
    }

    CHECK_FREAD;

    NodePtr n(createNode(t, nam));
    n->setAttentionValue(junk->getAttentionValue());
    n->setTruthValue(junk->getTruthValue());
    n->_uuid = junk->_uuid;

    handles->add(h, n);
    return n;
}

void SavingLoading::writeLink(FILE *f, LinkPtr link)
{
    logger().fine("writeLink(): %s", link->toString().c_str());

    writeAtom(f, link);

    // the link's arity is written on the file
    Arity arity = link->getArity();
    fwrite(&arity, sizeof(Arity), 1, f);

    // the link's outgoing set is written on the file
    for (Arity i = 0; i < arity; i++) {
        //logger().fine("writeLink(): outgoing[%d] => %p: %s", i, link->outgoing[i], link->outgoing[i]->toString().c_str());
        UUID uuid = link->getOutgoingAtom(i).value();
        fwrite(&uuid, sizeof(UUID), 1, f);
    }
}

void SavingLoading::writeAttentionValue(FILE *f, AttentionValuePtr attentionValue)
{
    AttentionValue::sti_t tempSTI = attentionValue->getSTI();
    AttentionValue::lti_t tempLTI = attentionValue->getLTI();
    AttentionValue::vlti_t tempVLTI = attentionValue->getVLTI();

    fwrite(&tempSTI, sizeof(AttentionValue::sti_t), 1, f);
    fwrite(&tempLTI, sizeof(AttentionValue::lti_t), 1, f);
    fwrite(&tempVLTI, sizeof(AttentionValue::vlti_t), 1, f);
}

void SavingLoading::writeTruthValue(FILE *f, TruthValuePtr tv)
{
    std::string tvStr = tv->toString();
    logger().fine( "SavingLoading::writeTruthValue() tvStr = %s\n", tvStr.c_str());
    TruthValueType type = tv->getType();
    int length = tvStr.size();

    fwrite(&type, sizeof(TruthValueType), 1, f);
    fwrite(&length, sizeof(int), 1, f);
    fwrite(tvStr.c_str(), sizeof(char), length, f);
}

AttentionValuePtr SavingLoading::readAttentionValue(FILE *f)
{
    AttentionValue::sti_t tempSTI;
    AttentionValue::lti_t tempLTI;
    AttentionValue::vlti_t tempVLTI;
    bool b_read = true;

    FREAD_CK(&tempSTI, sizeof(AttentionValue::sti_t), 1, f);
    FREAD_CK(&tempLTI, sizeof(AttentionValue::lti_t), 1, f);
    FREAD_CK(&tempVLTI, sizeof(AttentionValue::vlti_t), 1, f);

    CHECK_FREAD;
    return createAV(tempSTI, tempLTI, tempVLTI);
}

static const char* typeToStr(TruthValueType t) throw (InvalidParamException)
{
    switch (t) {
    case SIMPLE_TRUTH_VALUE:
        return "SIMPLE_TRUTH_VALUE";
    case COUNT_TRUTH_VALUE:
        return "COUNT_TRUTH_VALUE";
    case INDEFINITE_TRUTH_VALUE:
        return "INDEFINITE_TRUTH_VALUE";
    default:
        throw InvalidParamException(TRACE_INFO,
             "Invalid Truth Value type: '%d'.", t);
    }
}

static TruthValuePtr tv_factory(TruthValueType type, const char* tvStr);

static TruthValuePtr tv_factory(TruthValueType type, const char* tvStr)
{
    switch (type) {
    case SIMPLE_TRUTH_VALUE: {
        float mean, conf;
        sscanf(tvStr, "(stv %f %f)", &mean, &conf);
        return SimpleTruthValue::createTV(static_cast<strength_t>(mean),
            static_cast<count_t>(SimpleTruthValue::confidenceToCount(conf)));
    }
    case COUNT_TRUTH_VALUE: {
        float tmean, tcount, tconf;
        sscanf(tvStr, "(ctv %f %f %f)", &tmean, &tconf, &tcount);
        return CountTruthValue::createTV(
            static_cast<strength_t>(tmean),
            static_cast<confidence_t>(tconf),
            static_cast<count_t>(tcount));
    }
    case INDEFINITE_TRUTH_VALUE: {
        float m, l, u, c, d;
        int s;
        sscanf(tvStr, "[%f,%f,%f,%f,%f,%d]", &m, &l, &u, &c, &d, &s);
        IndefiniteTruthValuePtr result(
            IndefiniteTruthValue::createITV(static_cast<strength_t>(l),
                      static_cast<strength_t>(u),
                      static_cast<confidence_t>(c)));
        result->setDiff(static_cast<strength_t>(d));
        result->setSymmetric(s != 0);
        result->setMean(static_cast<strength_t>(m));
        return result;

    }
    default:
        throw InvalidParamException(TRACE_INFO,
            "Invalid Truth Value type in factory(...): '%d'.", type);
        break;
    }
    return NULL;
}

TruthValuePtr SavingLoading::readTruthValue(FILE *f)
{
    //logger().fine("SavingLoading::readTruthValue()");
    TruthValueType type;
    int length;
    bool b_read = true;

    FREAD_CK(&type, sizeof(TruthValueType), 1, f);
    FREAD_CK(&length, sizeof(int), 1, f);
    //logger().fine("SavingLoading::readTruthValue() type = %d, length =  %d", type, length);
    char *tvStr = new char[length+1];
    FREAD_CK(tvStr, sizeof(char), length, f);
    tvStr[length] = '\0';

    //logger().fine("SavingLoading::readTruthValue() tvStr = %s\n", tvStr);
    logger().info("SavingLoading::readTruthValue() tvStr = %s\n", tvStr);
    TruthValuePtr result = tv_factory(type, tvStr);
    delete[] tvStr;
    CHECK_FREAD;
    return result;
}

LinkPtr SavingLoading::readLink(FILE* f, Type t, HandMapPtr handles)
{
    logger().fine("SavingLoading::readLink()");

    AtomPtr junk(createLink(t, HandleSeq()));
    Handle h = readAtom(f, junk);

    // the link's arity is read from the file
    Arity arity;
    bool b_read = true;
    FREAD_CK(&arity, sizeof(Arity), 1, f);

    // the link's outgoing set is read from the file
    HandleSeq oset;
    for (Arity i = 0; i < arity; i++) {
        UUID uuid;
        FREAD_CK(&uuid, sizeof(UUID), 1, f);
        Handle h(uuid);
        oset.push_back( handles->get(h)->getHandle() );
    }
    LinkPtr link(createLink(t, oset));
    link->setAttentionValue(junk->getAttentionValue());
    link->setTruthValue(junk->getTruthValue());
    link->_uuid = junk->_uuid;
    handles->add(h, link);
    return link;
}

void SavingLoading::printProgress(const char *s, int n)
{
    static int old = -1;

    if (old != n) {
        old = n;
        logger().debug("Memory %s: %d%% done.", s, n);
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

void SavingLoading::loadRepositories(FILE *f, HandMapPtr conv) throw (RuntimeException)
{
    logger().fine("SavingLoading::loadRepositories");
    unsigned int size;
    size_t rc = fread(&size, sizeof(unsigned int), 1, f);

    if (rc != 1 or size != repositories.size()) {
        logger().error("Number of repositories in dump file (%d) is different from number of registered repositories (%d)", size, repositories.size());
        return;
    }

    for (unsigned int i = 0; i < size; i++) {
        int idSize;
        rc = fread(&idSize, sizeof(int), 1, f);
        if (rc != 1) { logger().error("Bad iidSize read, truncated. "); return; }

        std::unique_ptr<char> id(new char[idSize]);

        rc = fread(id.get(), sizeof(char), idSize, f);
        if (rc != (size_t)idSize) { logger().error("Bad id read, truncated. "); return; }

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
