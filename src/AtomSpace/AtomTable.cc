/**
 * AtomTable.cc
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */
#ifdef HAVE_LIBPTHREAD
#include <pthread.h>
#endif

#include <stdlib.h>
#include "AtomTable.h"
#include "ClassServer.h"
#include "exceptions.h"
#include "StatisticsMonitor.h"
#include "Link.h"
#include "Node.h"
#include "TLB.h"
extern "C" {
#include "md5.h"
}
//#include "NMPrinter.h"
#include "CoreUtils.h"
#include "AtomSpaceDefinitions.h"
#include "Logger.h"
#include "HandleMap.cc"

#define USE_ATOM_HASH_SET


int hashAtom::operator()(Atom* a) const{
//    printf("hashAtom a = %p\n", a);
//    printf("table of a = %d\n", a->getAtomTable());
//    printf("%s\n", a->toString().c_str());
    return(a->hashCode());
}

bool eqAtom::operator()(Atom* a1, Atom* a2) const{
//    printf("eqAtom a1 = %p, a2 = %p\n", a1, a2);
//    printf("table of a1 = %d\n",  a1->getAtomTable());
//    printf("table of a2 = %d\n",  a2->getAtomTable());
//    printf("a1 => %s\n", a1->toString().c_str());
//    printf("a2 => %s\n", a2->toString().c_str());
    return (a1->equals(a2));
}

AtomTable::AtomTable(bool dsa)
{
    useDSA = dsa;
    size = 0;
    atomSet = new AtomHashSet();

    // There are four indices. One for types, one for target types, 
    // one for names and one for importance ranges. The typeIndex
    // is NUMBER_OF_CLASSES+2 because NOTYPE is NUMBER_OF_CLASSES+1
    // and typeIndex[NOTYPE] is asked for if a typename is misspelled.
    // (because ClassServer::getType() returns NOTYPE in this case).
    typeIndex.resize(ClassServer::getNumberOfClasses()+2, UNDEFINED_HANDLE);
    targetTypeIndex.resize(ClassServer::getNumberOfClasses()+2, UNDEFINED_HANDLE);
    nameIndex.resize(NAME_INDEX_SIZE, UNDEFINED_HANDLE);
    importanceIndex.resize(IMPORTANCE_INDEX_SIZE, UNDEFINED_HANDLE);
    predicateIndex.resize(MAX_PREDICATE_INDICES, UNDEFINED_HANDLE);
    predicateHandles.resize(MAX_PREDICATE_INDICES, UNDEFINED_HANDLE);
    predicateEvaluators.resize(MAX_PREDICATE_INDICES, NULL); 
    numberOfPredicateIndices = 0;
    predicateHandles2Indices = new HandleMap<int>();

#ifdef HAVE_LIBPTHREAD
    pthread_mutex_init(&iteratorsLock, NULL);
#endif

}

AtomTable::~AtomTable()
{
#ifdef USE_ATOM_HASH_SET
    // remove all atoms from AtomTable
    AtomHashSet::iterator it = atomSet->begin();
    
    while(it != atomSet->end()) {
        //MAIN_LOGGER.log(Util::Logger::FINE, "Removing atom %s (atomSet size = %u)", (*it)->toString().c_str(), atomSet->size());
        remove(TLB::getHandle(*it), true);
        it = atomSet->begin();
    }
    atomSet->clear();
    delete (atomSet);
#endif
    delete (predicateHandles2Indices); 
}

bool AtomTable::isCleared() const{
//    tableId = id;
//    useDSA = dsa;
    if (size != 0) {
        //printf("AtomTable::size is not 0\n");
        return false;
    }

    if (atomSet->size() != 0) {
        //printf("AtomTable[%d]::atomSet is not empty. size =%d\n", tableId, atomSet->size());
        return false;
    }

    for (int i = 0; i < ClassServer::getNumberOfClasses(); i++) {
        if (typeIndex[i] != 0) {
            //printf("typeIndex[%d] is not 0\n", i);
            return false;
        }
        if (targetTypeIndex[i] != 0) {
            //printf("targetTypeIndex[%d] is not 0\n", i);
            return false;
        }
    }
    for (int i = 0; i < NAME_INDEX_SIZE; i++) {
        if (nameIndex[i] != 0) {
            //printf("nameIndex[%d] is not 0\n", i);
            return false;
        }
    }
    for (int i = 0; i < IMPORTANCE_INDEX_SIZE; i++) {
        if (importanceIndex[i] != 0) {
            //printf("importanceIndex[%d] is not 0\n", i);
            return false;
        }
    }
    for (int i = 0; i < MAX_PREDICATE_INDICES; i++) {
        if (predicateIndex[i] != 0) {
            //printf("predicateIndex[%d] is not 0\n", i);
            return false;
        }
        if (predicateHandles[i] != 0) {
            //printf("predicateHandles[%d] is not 0\n", i);
            return false;
        }
        if (predicateEvaluators[i] != 0) {
            //printf("predicateEvaluators[%d] is not 0\n", i);
            return false;
        }
    }
    
    if (numberOfPredicateIndices != 0) {
        //printf("numberOfPredicateIndices is not 0\n");
        return false;
    }
    if (predicateHandles2Indices->getCount() != 0) {
        //printf("predicateHandles2Indices is not empty\n");
        return false;
    }
    for (unsigned int i = 0; i < iterators.size(); i++) {
        if (iterators[i]->hasNext()) {
            //printf("iterators[%d] is not empty\n", i);
            return false;
        }
    }
    return true;
}

void AtomTable::addPredicateIndex(Handle predicateHandle, PredicateEvaluator* evaluator) 
                                                                throw (InvalidParamException) {

    if (numberOfPredicateIndices > MAX_PREDICATE_INDICES) {
        throw InvalidParamException(TRACE_INFO, 
              "AtomTable - Exceeded number of predicate indices = %d", MAX_PREDICATE_INDICES);
    }
    if (predicateHandles2Indices->contains(predicateHandle)) {
        throw InvalidParamException(TRACE_INFO, 
              "AtomTable - There is already an index for predicate handle %p", predicateHandle);
    }
    
    // Ok, add it.
    predicateHandles2Indices->add(predicateHandle, numberOfPredicateIndices);
    predicateHandles[numberOfPredicateIndices] = predicateHandle;
    predicateEvaluators[numberOfPredicateIndices] = evaluator;
    numberOfPredicateIndices++;
}

void AtomTable::registerIterator(HandleIterator* iterator) {

    lockIterators();
    iterators.push_back(iterator);
    unlockIterators();
}

void AtomTable::unregisterIterator(HandleIterator* iterator) throw (RuntimeException) {

    lockIterators();

    std::vector<HandleIterator*>::iterator it = iterators.begin();
    while (it != iterators.end()) {
        if (*it == iterator) {
            iterators.erase(it);
            unlockIterators();
            return;
        } else {
            it++;
        }
    }

    unlockIterators();
    throw RuntimeException(TRACE_INFO, "could not unregister iterator");
}

unsigned int AtomTable::strHash(const char* name) const{

    // this is a traditional hash algorithm that implements the MD5
    // XXX Why? Isn't MD5 a bit of overkill for this? 
    // It'll just be slow, without adding much value.

    // special hash value for NULL names
    if (name == NULL) return 0;

    unsigned int hash = 0;
    MD5_CTX context;
    unsigned char digest[16];

    MD5Init(&context);
    MD5Update(&context, (const unsigned char*) name, strlen(name));
    MD5Final(digest, &context);

    hash = *((int*) digest);
    hash ^= *(((int*) digest) + 1);
    hash ^= *(((int*) digest) + 2);
    hash ^= *(((int*) digest) + 3);

    return (hash % (NAME_INDEX_SIZE - 1)) + 1;
}

inline unsigned int AtomTable::getNameHash(Atom* atom) const
{
    return ClassServer::isAssignableFrom(NODE, atom->getType()) ?
          strHash(((Node*) atom)->getName().c_str()) : strHash(NULL);
}

unsigned int AtomTable::importanceBin(short importance) {
    // STI is in range of [-32768, 32767] so adding 32768 puts it in
    // [0, 65535] which is the size of the index
    return importance + 32768;
}

float AtomTable::importanceBinMeanValue(unsigned int bin) {
    return (float) ((((float) bin) + 0.5) / ((unsigned int) IMPORTANCE_INDEX_SIZE));
}

HandleEntry* AtomTable::makeSet(HandleEntry* set,
                                Handle head, int index) const
{
    while (TLB::isValidHandle(head)) {
        HandleEntry* entry = new HandleEntry(head);
        entry->next = set;
        set = entry;
        head = TLB::getAtom(head)->next(index);
    }

    return set;
}

Handle AtomTable::getTypeIndexHead(Type type) const
{
    return typeIndex[type];
}

Handle AtomTable::getTargetTypeIndexHead(Type type) const
{
    return targetTypeIndex[type];
}

Handle AtomTable::getNameIndexHead(const char* name) const{
    return nameIndex[strHash(name)];
}

Handle AtomTable::getPredicateIndexHead(int index) const{
    return predicateIndex[index];
}

PredicateEvaluator* AtomTable::getPredicateEvaluator(Handle gpnHandle) const{
    PredicateEvaluator* result = NULL;
    if (predicateHandles2Indices->contains(gpnHandle)) {
        int index = (int)((long) predicateHandles2Indices->get(gpnHandle));
        result = predicateEvaluators[index];
    }
    return result;
}

HandleEntry* AtomTable::findHandlesByGPN(const char* gpnNodeName, VersionHandle vh) const{
    //printf("AtomTable::findHandlesByGPN(%s)\n", gpnNodeName);
    // Get the GroundPredicateNode with such name
    Handle gpnHandle = getHandle(gpnNodeName, GROUNDED_PREDICATE_NODE);
    HandleEntry* result = findHandlesByGPN(gpnHandle);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::findHandlesByGPN(Handle gpnHandle, VersionHandle vh) const
{
    HandleEntry* result = NULL;    
    if (TLB::isValidHandle(gpnHandle)) {
        //printf("AtomTable::findHandlesByGPN(): found gnpHandle = %p\n", gpnHandle);
        if (predicateHandles2Indices->contains(gpnHandle)) {
            int index = (int)((long) predicateHandles2Indices->get(gpnHandle));
            //printf("AtomTable::findHandlesByGPN(): found index %d for gpnHandle\n", index);
            result = makeSet(NULL, getPredicateIndexHead(index), PREDICATE_INDEX | index);
        }
    }
    result = HandleEntry::filterSet(result, vh);
    return result;
}

Handle AtomTable::getHandle(const char* name, Type type) const{
    if (!ClassServer::isAssignableFrom(NODE, type)) {    
        return UNDEFINED_HANDLE;
    }
#ifdef USE_ATOM_HASH_SET
    Node* node = new Node(type, name);
    AtomHashSet::iterator it = atomSet->find(node);
    Handle result = UNDEFINED_HANDLE;
    if (it != atomSet->end()) {
        Atom* resultAtom = *it;
        result = TLB::getHandle(resultAtom);
    }
    delete node;
    return result;
#else    
    // creates a set with all atoms whose names have the same hash value as
    // the name key
    HandleEntry* set = makeSet(NULL, getNameIndexHead(name), NAME_INDEX);

    // the set is filtered by name and type
    set = HandleEntry::filterSet(set, name, type, false);

    // if any atom is left on the set, there exists a matching atom which is
    // returned
    Handle result = UNDEFINED_HANDLE;
    if (set != NULL) {
        result = set->handle;
        delete set;
    }
    return result;
#endif    
}

HandleEntry* AtomTable::buildSet(Type type, bool subclass, 
                                 Handle(AtomTable::*f)(Type) const,
                                 int index) const
{
    // Builds a set for the given type.
    HandleEntry* set = makeSet(NULL, (this->*f)(type), index);

    if (subclass) {
        // If subclasses are accepted, the subclasses are returned in the
        // array types. 
        int n;
        Type *types = ClassServer::getChildren(type, n);

        //printf("Checking %d subclasses:\n", n);
 
        // for all subclasses found, a set is concatenated to the answer set
        for (int i = 0; i < n; i++) {
            //printf("%d\n", i);
            if (index && TARGET_TYPE_INDEX){
                index = types[i] & TARGET_TYPE_INDEX;    
            }
            
            set = makeSet(set, (this->*f)(types[i]), index);
        }
        //printf("\n");
        delete[](types);
    }
   
    return set;
}

HandleEntry* AtomTable::getHandleSet(Type type, bool subclass) const
{
    HandleEntry* set = buildSet(type, subclass, 
                                &AtomTable::getTypeIndexHead, TYPE_INDEX);
    return set;
}

HandleEntry* AtomTable::getHandleSet(Type type, Type targetType,
                                     bool subclass, bool targetSubclass) const
{
    HandleEntry* set = buildSet(targetType, targetSubclass, 
                                &AtomTable::getTargetTypeIndexHead, 
                                TARGET_TYPE_INDEX | targetType);
    return HandleEntry::filterSet(set, type, subclass);
}

HandleEntry* AtomTable::getHandleSet(Handle handle, Type type, 
                                     bool subclass) const
{
    HandleEntry* set = TLB::getAtom(handle)->getIncomingSet();
    if (set != NULL) set = set->clone();
    set = HandleEntry::filterSet(set, type, subclass);
    // Also filter links that do not belong to this table
    //set = HandleEntry::filterSet(set, tableId); 
    return set;
}

HandleEntry* AtomTable::getHandleSet(const std::vector<Handle>& handles,
                                     Type* types,
                                     bool* subclasses,
                                     Arity arity,
                                     Type type,
                                     bool subclass) const
{
//printf("AtomTable::getHandleSet()\n");  

#ifdef USE_ATOM_HASH_SET    
  // Check if it is the special case of looking for an specific atom
  if (ClassServer::isAssignableFrom(LINK, type) && (arity == 0 || !handles.empty())) {
    //printf("special case\n");  
    bool hasAllHandles = true;
    for (int i = 0; hasAllHandles && i < arity; i++) {
        hasAllHandles = TLB::isValidHandle(handles[i]);
    }
    //printf("hasAllHandles = %d, subclass = %d\n", hasAllHandles, subclass);  
    if (hasAllHandles && !subclass) { 
        //printf("building link for lookup: type = %d, handles.size() = %d\n", type, handles.size());  
        Link* link = new Link(type, handles);
        AtomHashSet::iterator it = atomSet->find(link);
        Handle h = UNDEFINED_HANDLE;
        if (it != atomSet->end()) {
            h = TLB::getHandle(*it);
        }
        HandleEntry* result = NULL;
        if (TLB::isValidHandle(h)) {
            result = new HandleEntry(h);
        }
        delete link;
        //cprintf(NORMAL, "Returning HandleSet by using atom hash_set!\n");
        return result;
    }
  }
#endif

  if (ClassServer::isAssignableFrom(LINK, type) && (arity == 0)) {
    HandleEntry* result = getHandleSet(type, subclass);
    result = HandleEntry::filterSet(result, arity);
    return result;
  }
  
  std::vector<HandleEntry*> sets(arity, NULL);

  int countdown = 0;

  // builds a set for each element in the outgoing set. Empty sets are
  // counted to be removed a posteriori
    for (int i = 0; i < arity; i++) {
        if ((!handles.empty()) && TLB::isValidHandle(handles[i])) {
            sets[i] = TLB::getAtom(handles[i])->getIncomingSet()->clone();
            sets[i] = HandleEntry::filterSet(sets[i], handles[i], i, arity);
            // Also filter links that do not belong to this table
            //sets[i] = HandleEntry::filterSet(sets[i], tableId);
            if (sets[i] == NULL) {
                for (int j = 0; j < i; j++) {
                    delete sets[j];
                }
                return NULL;
            }
        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            bool sub = subclasses == NULL ? false : subclasses[i];
            sets[i] = getHandleSet(type, types[i], subclass, sub);
            // Also filter links that do not belong to this table
            //sets[i] = HandleEntry::filterSet(sets[i], tableId);
            if (sets[i] == NULL) {
                for (int j = 0; j < i; j++) {
                    delete sets[j];
                }
                return NULL;
            }
        } else {
            countdown++;
        }
    }
    
    int newLength = arity;
    // if the empty set counter is not zero, removes them by shrinking the
    // list of sets
    if (countdown > 0) {
        //printf("newset allocated size = %d\n", (arity - countdown));
    // TODO: Perhaps it's better to simply erase the NULL entries of the sets 
        std::vector<HandleEntry*> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i] != NULL) {
                newset.push_back(sets[i]);
            }
        }
        sets = newset;
    }
    //printf("newLength = %d\n", newLength);

    if ((type != ATOM) || (!subclass)) {
        for (int i = 0; i < newLength; i++) {
            // filters by type and subclass in order to remove unwanted elements.
            // This is done before the intersection method to reduce the number of
            // elements being passed (intersection uses qsort, which is n log n)
            sets[i] = HandleEntry::filterSet(sets[i], type, subclass);
        }
    }

    // computes the intersection of all non-empty sets
    HandleEntry* set = HandleEntry::intersection(sets);
    // TODO: Why not move this filtering to the begining... Pehaps it will filter more before the intersection (which seems to be the most expensive operation)
    // filters the answer set for every type in the array of target types
    if (types != NULL) {
        for (int i = 0; i < arity; i++) {
            if (types[i] != NOTYPE) {
                bool sub = subclasses == NULL ? false : subclasses[i];
                set = HandleEntry::filterSet(set, types[i], sub, i, arity);
            }
        }
    }

    return set;
}

HandleEntry* AtomTable::getHandleSet(const char* name, Type type, bool subclass) const{
    // a list of the given names is built.
    HandleEntry* set = makeSet(NULL, getNameIndexHead(name), NAME_INDEX);

    // then the undesired names, because of a hash table conflict, are removed
    // from the list by filtering it.
    set = HandleEntry::filterSet(set, name);
    return HandleEntry::filterSet(set, type, subclass);
}

HandleEntry* AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass) const{

    // Gets the exact atom with the given name and type, in any AtomTable. 
    Handle handle = getHandle(targetName, targetType);

    HandleEntry* result = NULL;
    // then, if the atom returend is valid, the list with the given target name
    // and types will be returned.
    if (TLB::isValidHandle(handle)) {
        result = getHandleSet(handle, type, subclass);
    }
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass) const throw (RuntimeException) {
    //printf("getHandleSet begin\n");

    std::vector<HandleEntry*> sets(arity, NULL);

    int countdown = 0;
    // a list for each array of names is built. Then, it's filtered by the 
    // name (to avoid hash conflicts) and by the correspondent type in the
    // array of types.
    for (int i = 0; i < arity; i++) {
            //printf("getHandleSet: arity %d\n", i);
        bool sub = subclasses == NULL ? false : subclasses[i];
        if ((names != NULL) && (names[i] != NULL)) {
            if ((types != NULL) && (types[i] != NOTYPE)) {
                sets[i] = getHandleSet(names[i], types[i], type, subclass);
                if (sub){
                    // if subclasses are accepted, the subclasses are returned in the
                    // array types. 
                    int n;
    
                    Type *subTypes = ClassServer::getChildren(types[i], n);
                    
                    // for all subclasses found, a set is concatenated to the answer set
                    for (int j = 0; j < n; j++) {
                        HandleEntry *subSet = getHandleSet(names[i], subTypes[j], type, subclass);
                        sets[i] = HandleEntry::concatenation(sets[i], subSet);
                    }
                    delete[](subTypes);
                }
                sets[i] = HandleEntry::filterSet(sets[i], names[i], types[i], sub, i, arity);
            } else {
                for (int j = 0; j < i; j++) {
                    delete sets[j];
                }
                throw RuntimeException(TRACE_INFO, "Cannot make this search using only target name!\n");
            }
        } else if ((types != NULL) && (types[i] != NOTYPE)) {
            sets[i] = getHandleSet(type, types[i], subclass, sub);
            sets[i] = HandleEntry::filterSet(sets[i], types[i], sub, i, arity);
        } else {
            countdown++;
        }
    }

    // if the empty set counter is not zero, removes them by shrinking the
    // list of sets
    if (countdown > 0) {
        //printf("newset allocated size = %d\n", (arity - countdown));
    // TODO: Perhaps it's better to simply erase the NULL entries of the sets 
        std::vector<HandleEntry*> newset;
        for (int i = 0; i < arity; i++) {
            if (sets[i] != NULL) {
                newset.push_back(sets[i]);
            }
        }
        sets = newset;
    }

    // the intersection is made for all non-empty sets, and then is filtered 
    // by the optional specified type. Also, if subclasses are not accepted, 
    // it will not pass the filter.
    //printf("getHandleSet: about to call intersection\n");
    HandleEntry* set = HandleEntry::intersection(sets);
    //printf("getHandleSet: about to call filterSet\n");
//    return  HandleEntry::filterSet(set, type, subclass); // This filter redundant, since all getHandleSet above uses type and subclass
    return  set;
}

HandleEntry* AtomTable::getHandleSet(AttentionValue::sti_t lowerBound, AttentionValue::sti_t upperBound) const{

    // the indice for the lower bound and upper bound lists is returned.
    int lowerBin = importanceBin(lowerBound);
    int upperBin = importanceBin(upperBound);

    // the list of atoms with its importance equal to the lower bound is
    // returned.
    HandleEntry* set = makeSet(NULL, importanceIndex[lowerBin], IMPORTANCE_INDEX);

    // for the lower bound and upper bound index, the list is filtered, because
    // there may be atoms that have the same importanceIndex and whose 
    // importance is lower than lowerBound or bigger than upperBound.
    set = HandleEntry::filterSet(set, lowerBound, upperBound);

    if (lowerBin == upperBin) {
        // If both lower and upper bounds are in the same bin, 
        // it can ans must return the already built set. 
        // Otherwise, it will duplicate entries when concatening the upper set latter.
        return set;
    }

    // for every index within lowerBound and upperBound, the list is 
    // concatenated.
    while (++lowerBin < upperBin) {
        set = makeSet(set, importanceIndex[lowerBin], IMPORTANCE_INDEX);
    }

    // the list for the upperBin index is built and filtered.
    HandleEntry* uset = makeSet(NULL, importanceIndex[upperBin], IMPORTANCE_INDEX);
    uset = HandleEntry::filterSet(uset, lowerBound, upperBound);

    // then the two lists built are concatenated.
    return HandleEntry::concatenation(uset, set);
}

HandleEntry* AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass) const{
    return getHandleSet((const char**) NULL, types, subclasses, arity, type, subclass);
}


void AtomTable::merge(Atom *original, Atom *copy) {
    original->merge( copy);
    delete copy;
}
    
Handle AtomTable::add(Atom *atom) throw (RuntimeException)
{
    if (atom->getAtomTable() != NULL){
        //Atom is already inserted
        return  TLB::addAtom(atom);
    }
    Handle existingHandle = UNDEFINED_HANDLE;
    Node * nnn = dynamic_cast<Node *>(atom);
    Link * lll = dynamic_cast<Link *>(atom);
    if (nnn) {
        // checks if the node handle already exists.
        existingHandle = getHandle(nnn->getName().c_str(), atom->getType());
    } else if (lll) {
        // New link may already exist.
        std::vector<Handle> outgoing(atom->getArity());
        for(int i = 0; i < atom->getArity(); i++) {
             outgoing[i] = atom->getOutgoingSet()[i];
        }
        HandleEntry* head = getHandleSet(outgoing, NULL, NULL, atom->getArity(), atom->getType(), false);
        // if the link handle already exists.
        if (head != NULL) {
            // gets the existing handle
            existingHandle = head->handle;
        }
        delete(head);
    }
    if (TLB::isValidHandle(existingHandle)) {
        //printf("Merging existing Atom with the Atom being added ...\n");
        merge(TLB::getAtom(existingHandle), atom);
        return existingHandle;
    }

    // New atom, its Handle will be stored in the AtomTable

    // increments the size of the table
    size++;
    
#ifdef USE_ATOM_HASH_SET    
    // Adds to the hash_set
    //printf("Inserting atom %p in hash_set (type=%d, hashCode=%d)\n", atom, atom->getType(), atom->hashCode()); 
/*    
    printf("INSERTING ATOM (%p, type=%d, arity=%d) INTO ATOMSET:\n", atom, atom->getType(), atom->getArity());
    NMPrinter p;
    p.print(atom);
*/    
//    printf("%s\n", atom->toString().c_str());
    atomSet->insert(atom);
//    printf("AtomTable[%d]::atomSet->insert(%p) => size = %d\n", tableId, atom, atomSet->size());
#endif    

    // checks for null outgoing set members
    const std::vector<Handle>& ogs = atom->getOutgoingSet();
    for (int i = atom->getArity() - 1; i >= 0; i--) {
        if (TLB::isInvalidHandle(ogs[i])) {
            throw RuntimeException(TRACE_INFO, 
                  "AtomTable - Attempting to insert atom with invalid (null) outgoing members");
        }
    }

    Handle handle = TLB::addAtom(atom);

    // Inserts atom in the type index of its type (as head of the list)
    Type type = atom->getType();
    atom->setNext(TYPE_INDEX, typeIndex[type]);
    typeIndex[type] = handle; 

    // if the atom is a link, the targetIndexTypes list is built. Then, from 
    // the atom's arity, it will be checked how many targetTypes are distinct.
    int distinctSize;
    Type* targetTypes = atom->buildTargetIndexTypes(&distinctSize);
    if (distinctSize > 0) {
        // here, the atom is placed on each target index list. 
        Handle* targetIndices = new Handle[distinctSize];
        for (int i = 0; i < distinctSize; i++) {
            // Insert it as head of the corresponding list
            targetIndices[i] = targetTypeIndex[targetTypes[i]];
            targetTypeIndex[targetTypes[i]] = handle;
        }
        atom->setNextTargetTypeIndex(targetIndices);
    }
    delete[](targetTypes);

    // the atom is placed on its proper name index list.
    unsigned int nameHash = getNameHash(atom);
    atom->setNext(NAME_INDEX, nameIndex[nameHash]);
    nameIndex[nameHash] = handle;

    // the atom is placed on its proper importance index list.
    int bin = importanceBin(atom->getAttentionValue().getSTI());
    //printf("Adding handle %p with importance %f into importanceIndex (bin = %d)\n", atom, atom->getImportance(), bin);
    atom->setNext(IMPORTANCE_INDEX, importanceIndex[bin]);
    importanceIndex[bin] = handle;
    
    // Checks Atom against predicate indices and inserts it if needed
    for (int i = 0; i < numberOfPredicateIndices; i++) {
        //printf("Processing predicate index %d\n");
        PredicateEvaluator* evaluator = predicateEvaluators[i];
        //printf("Evaluating handle %p with PredicateEvaluator  = %p\n", handle, evaluator);
        if (evaluator->evaluate(handle)) {
            //printf("ADDING HANDLE %p TO THE PREDICATE INDEX %d (HEAD = %p)\n", handle, i, getPredicateIndexHead(i));
            atom->addNextPredicateIndex(i, predicateIndex[i]);
            predicateIndex[i] = handle; // adds as head of the linked list
/*
            printf("HEAD AFTER INSERTION = %p\n", getPredicateIndexHead(i));
            HandleEntry* indexedHandles = makeSet(NULL, getPredicateIndexHead(i), PREDICATE_INDEX | i);
            printf("Handles in the index %d: \n", i);
            while (indexedHandles != NULL) {
                printf("%p (%d)\t", indexedHandles->handle, TLB::getAtom(indexedHandles->handle)->getType());
                indexedHandles = indexedHandles->next;
            }
            printf("\n");
*/            
        }
    }

    // updates incoming set of all targets.
    for (int i = 0; i < atom->getArity(); i++) {
        atom->getOutgoingAtom(i)->addIncomingHandle(handle);
    }

    // updates statistics
    //float heat = atom->getHeat();
    //atom->rawSetHeat(0);
    //atom->setHeat(heat);

    atom->setAtomTable(this);

    if (useDSA){
        StatisticsMonitor::getInstance()->add(atom);
    }

    return handle;
}

bool AtomTable::updateImportanceIndex(Atom* atom, int bin) {

    // current receives the first element of the list that the atom is in.
    Handle current = importanceIndex[bin];
    Handle wanted = TLB::getHandle(atom);

    // checks if current is valid.
    if (TLB::isInvalidHandle(current)) {
        return(false);
    }
    // here is checked if the atom is on the first position of its importance
    // index list.
    if (current == wanted) {
        // if so, the new first element will be the next one.    
        importanceIndex[bin] = atom->next(IMPORTANCE_INDEX);
    } else {
        // if not, the list will be scanned until the atom is found.
        Handle p;
        while ((p = TLB::getAtom(current)->next(IMPORTANCE_INDEX)) != wanted) {
            current = p;
            if (TLB::isInvalidHandle(p)) {
                return(false);
            }
        }
        TLB::getAtom(current)->setNext(IMPORTANCE_INDEX, TLB::getAtom(wanted)->next(IMPORTANCE_INDEX));
    }    

    // the atom is placed on the last position of the new list.
    atom->setNext(IMPORTANCE_INDEX, importanceIndex[importanceBin(atom->getAttentionValue().getSTI())]);
    importanceIndex[importanceBin(atom->getAttentionValue().getSTI())] = TLB::getHandle(atom);
    return(true);
}

int AtomTable::getSize() const{
    return(size);
}

void AtomTable::print(std::ostream& output, Type type, bool subclass) const {
#ifdef USE_ATOM_HASH_SET
    for(AtomHashSet::const_iterator it = atomSet->begin(); it != atomSet->end(); it++) {
        Atom* atom = *it;
        bool matched = (subclass && ClassServer::isAssignableFrom(type, atom->getType())) || type == atom->getType();
        if (matched) output << atom << ": " << atom->toString() << endl;
    }
#else
    output << "Sorry, AtomTable::print() method is not implemented when USE_ATOM_HASH_SET is disabled" << endl;
#endif
}

HandleEntry* AtomTable::extract(Handle handle, bool recursive) {
    HandleEntry* result = NULL;
    //printf("AtomTable::extract(%p)\n", handle);

    // TODO: Check if this atom is really inserted in this AtomTable and get the exact Atom object  
    
    Atom *atom = TLB::getAtom(handle);
    //MAIN_LOGGER.log(Util::Logger::FINE, "AtomTable::extract(): atom = %s", atom->toString().c_str());
    //MAIN_LOGGER.log(Util::Logger::FINE, "AtomTable::extract(): atom = %p", atom);

    // If the atom is already marked for removal, we should not process it again
    if (atom->isMarkedForRemoval()) {
        return result;
    }
    atom->markForRemoval();
    
    // if recursive-flag is set, also extract all the links in the atom's incoming set
    if (recursive) {
        //printf("AtomTable::extract() recursive\n");
        HandleEntry* incomingSet = atom->getIncomingSet();
        //MAIN_LOGGER.log(Util::Logger::FINE, "AtomTable::extract(): incomingSet = %s", incomingSet?incomingSet->toString().c_str():"NULL");
        //printf("incomingSet = %s\n", incomingSet->toString().c_str());
	std::vector<Handle> hs; 
        for (HandleEntry* in = incomingSet; in != NULL; in=in->next) {
            hs.push_back(in->handle);
        }
        for(std::vector<Handle>::iterator it = hs.begin(); it != hs.end(); it++) {
            Handle h = *it;
            //printf("Following handle in incomingSet: %p\n", h);
            if(!TLB::getAtom(h)->isMarkedForRemoval()) {
                HandleEntry* inResult = extract(h, true);
                if (inResult) {
                    result = HandleEntry::concatenation(inResult, result);
                    //printf("result = %s\n", result?result->toString().c_str():"NULL");
                } else {
                    MAIN_LOGGER.log(Util::Logger::ERROR, "AtomTable.extract(): extract() applied to an incoming set's element, which was not marked for removal, returned no entry: \n", TLB::getAtom(h)->toShortString().c_str());
                }
            }
        }
    }
    if (atom->getIncomingSet()) {
        MAIN_LOGGER.log(Util::Logger::WARNING, "AtomTable.extract(): attempting to extract atom with non-empty incoming set: %s\n", atom->toShortString().c_str());
        for (HandleEntry* it = atom->getIncomingSet(); it != NULL; it = it->next) {
            MAIN_LOGGER.log(Util::Logger::WARNING, "\t%s\n", TLB::getAtom(it->handle)->toShortString().c_str());
        }
        atom->unsetRemovalFlag();
        return result;
    }
    
    //decrements the size of the table
    size--;
    
#ifdef USE_ATOM_HASH_SET    
    //Extracts atom from hash_set
    //printf("Removing atom %p in hash_set (type=%d, hashCode=%d) from atomSet %p\n", atom, atom->getType(), atom->hashCode(), atomSet); 

/*    
    printf("REMOVING ATOM (%p) FROM ATOMSET:\n", atom);
    NMPrinter p;
    p.print(atom);
//    printf("%s\n", atom->toString().c_str());
    int s1 = atomSet->size();
    AtomHashSet::iterator it = atomSet->find(atom);
    if (it == atomSet->end()) {
        printf("ATOM TO BE EXTRACTED NOT FOUND IN ATOMSET: %s\n", atom->toString().c_str());
    }
*/ 
    atomSet->erase(atom);
//    printf("AtomTable[%d]::atomSet->erase(%p) => size = %d\n", tableId, atom, atomSet->size());
/*    
    int s2 = atomSet->size();
    if (s2 == s1) {
        printf("ATOM EXTRACTED FROM ATOMSET BUT DIDNT MAKE IT DECREASE ITS SIZE: %s\n", atom->toString().c_str());
    }
*/    
#endif

        //printf("AtomTable::extract() Before clearing attributes\n");

    // forces values to zero to update statistics correctly
    //atom->setImportance(0); // TODO: Performance question: Can this be done after removing from index?
    //atom->setHeat(0);

        //printf("AtomTable::extract() After clearing attributes\n");
        
    if (useDSA){
        // updates all global statistics regarding the removal of this atom
        StatisticsMonitor::getInstance()->remove(atom);
    }

    //MAIN_LOGGER.log(Util::Logger::FINE, "AtomTable::extract(): Before removing indices: atom = %p", atom);
    // remove from indices
    removeFromIndex(atom, typeIndex, TYPE_INDEX, atom->getType());
    unsigned int nameHash = getNameHash(atom);
    removeFromIndex(atom, nameIndex, NAME_INDEX, nameHash);
    int bin = importanceBin(atom->getAttentionValue().getSTI());
    removeFromIndex(atom, importanceIndex, IMPORTANCE_INDEX, bin);
    removeFromTargetTypeIndex(atom);
    removeFromPredicateIndex(atom);

    //printf("AtomTable::extract(): after removing indices\n");
    
    // remove from incoming sets
    Handle atomHandle = TLB::getHandle(atom);

    int arity = atom->getArity();
    for (int i = 0; i < arity; i++) {
        Atom* target = atom->getOutgoingAtom(i);
        target->removeIncomingHandle(atomHandle);
    }
    
    //printf("AtomTable::extract(): after removing from Network\n");
    
    // remove from iterators
    lockIterators();
    for (unsigned int i = 0; i < iterators.size(); i++) {
        // TODO: CAN THIS REALLY BE CALLED AFTER THE ATOM HAS BEEN REMOVED FROM TYPE INDEX ALREADY ?
        removeFromIterator(atom, iterators[i]);
    }
    unlockIterators();

    //printf("AtomTable::extract(): after removing from Iterators\n");

    result = HandleEntry::concatenation(new HandleEntry(atomHandle), result);
    
    return result;
}

bool AtomTable::remove(Handle handle, bool recursive) {
    HandleEntry* extractedHandles = extract(handle, recursive);
    if (extractedHandles) {
        removeExtractedHandles(extractedHandles);
        return true;
    }
    return false; 
} 
    
void AtomTable::removeExtractedHandles(HandleEntry* extractedHandles) {    
    if (extractedHandles) {
        HandleEntry* currentEntry = extractedHandles;
        while (currentEntry) {
            Handle h = currentEntry->handle;
            Atom* atom = TLB::getAtom(h);
            TLB::removeAtom(atom);
            delete atom;
            currentEntry = currentEntry->next;
        }
        delete extractedHandles;
    }
}

void AtomTable::removeFromIndex(Atom *victim, std::vector<Handle>& index, int indexID, int headIndex) throw (RuntimeException) {
    //MAIN_LOGGER.log(Util::Logger::FINE, "AtomTable::removeFromIndex(): index.size() = %d, indexId = %d(%x), headIndex = %d(%x)", index.size(), indexID, indexID, headIndex, headIndex);
    Handle victimHandle = TLB::getHandle(victim);
    //MAIN_LOGGER.log(Util::Logger::FINE, "victim = %s", victim?victim->toString().c_str():"NULL");

    Handle p = index[headIndex];
    Handle q = UNDEFINED_HANDLE;
    while (p != victimHandle) {
        if (TLB::isInvalidHandle(p)) {
            throw RuntimeException(TRACE_INFO, 
                  "AtomTable - Unable to remove atom. NULL atom at index 0x%X.", indexID);
        }
        Atom *patom = TLB::getAtom(p);
        //MAIN_LOGGER.log(Util::Logger::FINE, "Next atom in index = %s", patom?patom->toString().c_str():"NULL");
        q = p;
        p = patom->next(indexID);
    }
    
    //cprintf(DEBUG,"removeFromIndex(): found position in the index\n");
    
    if (TLB::isInvalidHandle(q)) {
        index[headIndex] = victim->next(indexID);
    } else {
        Atom *qatom = TLB::getAtom(q);
        Atom *patom = TLB::getAtom(p);
        qatom->setNext(indexID, patom->next(indexID));
    }

    victim->setNext(indexID, UNDEFINED_HANDLE);
}

void AtomTable::removeFromTargetTypeIndex(Atom *atom) {
    //MAIN_LOGGER.log(Util::Logger::FINE, "AtomTable::removeFromTargetTypeIndex(%p)", atom);

    int arraySize;
    Type *types = atom->buildTargetIndexTypes(&arraySize);

    for (int i = 0; i < arraySize; i++) {
        removeFromIndex(atom, targetTypeIndex, TARGET_TYPE_INDEX | types[i], types[i]);
    }

    delete[](types);
}

void AtomTable::removeFromPredicateIndex(Atom *atom) {
    if (!atom->hasPredicateIndexInfo()) {
        //MAIN_LOGGER.log(Util::Logger::FINE, "removeFromPredicateIndex(%p): No predicate index info", atom);
        return;
    }
    //MAIN_LOGGER.log(Util::Logger::FINE, "removeFromPredicateIndex(%p): has predicate index info", atom);
    
    int arraySize;
    int *predicateIndices = atom->buildPredicateIndices(&arraySize);

    //cprintf(DEBUG,"Found %d predicate indices\n", arraySize);
    for (int i = 0; i < arraySize; i++) {
        //cprintf(DEBUG,"removing from index %d => %p\n", predicateIndices[i], PREDICATE_INDEX | predicateIndices[i]);
        removeFromIndex(atom, predicateIndex, PREDICATE_INDEX | predicateIndices[i], predicateIndices[i]);
    }

    delete[](predicateIndices);
}

void AtomTable::decayShortTermImportance() throw (RuntimeException) {

    std::vector<Handle> clone(importanceIndex);

    for (unsigned int band = 0; band < (unsigned int) IMPORTANCE_INDEX_SIZE; band++) {
        Handle current = importanceIndex[band];
        Handle previous;
        if (current == clone[band]) {
            // head has not changed
            previous = UNDEFINED_HANDLE;
        } else {
            // head has changed: seek correct previous
            previous = clone[band];
            while (TLB::getAtom(previous)->next(IMPORTANCE_INDEX) != current) {
                previous = TLB::getAtom(previous)->next(IMPORTANCE_INDEX);
                if (TLB::isInvalidHandle(previous)) {
                    throw RuntimeException(TRACE_INFO, "AtomTable - Found null previous");
                }
            }
        }
        while (TLB::isValidHandle(current)) {
            // the importance is updated.
            Atom* atom = TLB::getAtom(current);
            decayAtomShortTermImportance(atom);
            unsigned int newBand = AtomTable::importanceBin(atom->getAttentionValue().getSTI());
            // if the atom has to be reindexed, it will be placed on the first
            // position of the new list, and the previous element in the old 
            // list will point to old next of the atom. In case the atom is the
            // first of the list, its next will be the new head of the list.
            if (newBand != band) {
                if (TLB::isInvalidHandle(previous)) {
                    // element in head
                    clone[band] = atom->next(IMPORTANCE_INDEX);
                } else {
                    // element in body
                    TLB::getAtom(previous)->setNext(IMPORTANCE_INDEX, atom->next(IMPORTANCE_INDEX));
                }
                Handle previousHead = clone[newBand];
                clone[newBand] = current;
                current = atom->next(IMPORTANCE_INDEX);
                atom->setNext(IMPORTANCE_INDEX, previousHead);
                StatisticsMonitor::getInstance()->atomChangeImportanceBin(atom->getType(), band, newBand);
            } else {
                previous = current;
                current = atom->next(IMPORTANCE_INDEX);
            }
        }
    }

    // clone is copied to the real importance index lists.
    importanceIndex = clone;
}

void AtomTable::decayAtomShortTermImportance(Atom* atom) {
    atom->getAVPointer()->decaySTI();
}


void AtomTable::removeFromIterator(Atom *atom, HandleIterator *iterator) {
    if (iterator->currentHandle == TLB::getHandle(atom)) {
        iterator->next();
    }
}
void AtomTable::lockIterators() {
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_lock(&iteratorsLock);
#endif
}

void AtomTable::unlockIterators() {
#ifdef HAVE_LIBPTHREAD
    pthread_mutex_unlock(&iteratorsLock);
#endif
}


HandleIterator* AtomTable::getHandleIterator(){
    return new HandleIterator(this, (Type)ATOM, true);
}

HandleIterator* AtomTable::getHandleIterator(Type type, bool subclass, VersionHandle vh){
    return new HandleIterator(this, type, subclass, vh);
}

Handle AtomTable::getImportanceIndexHead(int i) const{
    return importanceIndex[i];
}

bool AtomTable::usesDSA() const{
    return useDSA;
}


HandleEntry* AtomTable::getHandleSet(Type type, bool subclass, 
                                     VersionHandle vh) const
{
    //printf("AtomTable::getHandleSet(Type =%d, bool=%d, AtomTableList=%d)\n", type, subclass, tableId);
    //printf("About to call AtomTable::getHandleSet()\n");
    HandleEntry* result = this->getHandleSet(type, subclass);
    //printf("Got handles from AtomTable\n");
    result = HandleEntry::filterSet(result, vh);
    //printf("Returning %p\n", result);
    return result;
}

HandleEntry* AtomTable::getHandleSet(Type type, Type targetType, bool subclass, bool targetSubclass, VersionHandle vh, VersionHandle targetVh) const{
    //printf("AtomTable::getHandleSet(Type type, Type targetType, bool subclass, bool targetSubclass, VersionHandle vh, VersionHandle targetVh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(type, targetType, subclass, targetSubclass);
    result = HandleEntry::filterSet(result, vh);
    result = HandleEntry::filterSet(result, targetType, targetSubclass, targetVh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(Handle handle, Type type, bool subclass, VersionHandle vh) const{
    //printf("AtomTable::getHandleSet(Handle handle, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(handle, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const std::vector<Handle>& handles, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const{
    //printf("AtomTable::getHandleSet(const std::vector<Handle>& handles, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(handles, types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char* name, Type type, bool subclass, VersionHandle vh) const{
    //printf("AtomTable::getHandleSet(const char* name, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = NULL;
    if (name == NULL) { 
        result = getHandleSet(type, subclass, vh); 
    } else {
        result = this->getHandleSet(name, type, subclass);
        result = HandleEntry::filterSet(result, vh);
    }
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass, VersionHandle vh, VersionHandle targetVh) const{
    //printf("AtomTable::getHandleSet(const char* targetName, Type targetType, Type type, bool subclass, VersionHandle vh, VersionHandle targetVh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(targetName, targetType, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    result = HandleEntry::filterSet(result, targetName, targetType, targetVh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const{
    //printf("AtomTable::getHandleSet(const char** names, Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(names, types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

HandleEntry* AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh) const{
    //printf("AtomTable::getHandleSet(Type* types, bool* subclasses, Arity arity, Type type, bool subclass, VersionHandle vh, AtomTableList tableId\n");
    HandleEntry* result = this->getHandleSet(types, subclasses, arity, type, subclass);
    result = HandleEntry::filterSet(result, vh);
    return result;
}

/*
 * If this method is needed it needs to be refactored to use AttentionValue instead of floats
HandleEntry* AtomTable::getHandleSet(float lowerBound, float upperBound, VersionHandle vh) const{
    //printf("AtomTable::getHandleSet(float lowerBound, float upperBound, VersionHandle vh, AtomTableList tableId)\n");
    HandleEntry* result = this->getHandleSet(lowerBound, upperBound);
    result = HandleEntry::filterSet(result, vh);
    return result;
}
*/

