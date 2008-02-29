/**
 * Atom.cc
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#include "Atom.h"
#include "Link.h"
#include "ClassServer.h"
#include "Defaults.h"
#include "StatisticsMonitor.h"
#include "exceptions.h"
#include "TLB.h"
#include "utils.h"
#include "AtomTable.h"
#include "AtomSpaceDefinitions.h"
#include <set>
#ifndef WIN32
#include <unistd.h>
#endif
//#define USE_SHARED_DEFAULT_TV

#undef Type

void Atom::init(Type type, const std::vector<Handle>& outgoing, const TruthValue& tv ) {
    // resets all flags
    flags = 0;

    atomTable = NULL;

#ifndef USE_STD_VECTOR_FOR_OUTGOING
    this->outgoing = NULL;
    this->arity = 0; // not really needed
#endif
    this->incoming = NULL;
    this->type = type;
    setOutgoingSet(outgoing); // need to call the method to handle specific subclass case

    //// sets default values
    //rawSetHeat(Defaults::getDefaultHeat(type));
    //rawSetImportance(Defaults::getDefaultImportance(type));

    // resets all indices
    indices = (Handle*) calloc(NUMBER_OF_INDICES, sizeof(Handle));
    //memset(indices, 0, sizeof(Handle) * NUMBER_OF_INDICES);

    // this variable is an array; each position is a pointer to the next
    // element in a target type list that the atom is in
    targetTypeIndex = NULL;

    predicateIndexInfo = NULL;

    //attentionValue = (AttentionValue*)&AttentionValue::getDefaultAV();
    attentionValue = (AttentionValue*) AttentionValue::factory();

#ifdef USE_SHARED_DEFAULT_TV
    truthValue = NULL; 
    setTruthValue(tv);
#else 
    truthValue = tv.isNullTv()? TruthValue::DEFAULT_TV().clone() : tv.clone();
#endif     
}

Atom::Atom(Type type, const std::vector<Handle>& outgoingVector, const TruthValue& tv ) {
    init(type, outgoingVector, tv);
}

Atom::~Atom() throw (RuntimeException) {
    // checks if there is still an atom pointing to the one being removed
    if (incoming != NULL) {
        throw RuntimeException(TRACE_INFO, "Attempting to remove atom with non-empty incoming set.");
    }

#ifndef USE_STD_VECTOR_FOR_OUTGOING
    //printf("Atom::~Atom() freeing outgoing\n");
    if (outgoing) free(outgoing);
#endif        
    //printf("Atom::~Atom() deleting targetTypeIndex\n");
    delete[](targetTypeIndex);
    //printf("Atom::~Atom() deleting predicateIndexInfo\n");
    delete(predicateIndexInfo);
    //printf("Atom::~Atom() deleting truthValue\n");
    delete (attentionValue);
#ifdef USE_SHARED_DEFAULT_TV    
    if (truthValue != &(TruthValue::DEFAULT_TV())) {
        delete truthValue;
    }
#else
    delete truthValue;
#endif    
    free(indices);
    //printf("Atom::~Atom() end\n");
}

bool Atom::isReal() const{
    long value = (long)this;
    bool real = (value < 0 || value >= NUMBER_OF_CLASSES);
   //if(!real){
        //fprintf(stdout, "Atom - Type: %d, Pointe converted: (%p, %d)\n", type, this, (long)this);
        //fflush(stdout);
    //}
    return real;
}

//float Atom::getHeat() {

//    return ShortFloatOps::getValue(&heat);
//}

//void Atom::rawSetHeat(float value) {

//    // if the value is out of bounds, it is set to either the upper or lower bound
//    if (value > 1) value = 1;
//    if (value < 0) value = 0;

//    ShortFloatOps::setValue(&heat, value);
//}

//void Atom::setHeat(float value) {
//    float old = getHeat();
//    rawSetHeat(value);
//    StatisticsMonitor::getInstance()->updateHeatSummation(type, getHeat() - old);
//}

// THIS IS NOT PUBLIC API - don't use if you don't know exactly what you're doing
// returned AttentionValue object is supposed to be const, as in the method
// getAttentionValue() below. Don't use this method to change AttentionValue
// because you may break AtomTable indices.
AttentionValue* Atom::getAVPointer() {
    return attentionValue;
}

const AttentionValue& Atom::getAttentionValue() const{
    return *attentionValue;
}

const TruthValue& Atom::getTruthValue() const{
    return *truthValue;
}

void Atom::setTruthValue(const TruthValue& tv) {
#ifdef USE_SHARED_DEFAULT_TV   
    if (truthValue != NULL && &tv != truthValue && truthValue != &(TruthValue::DEFAULT_TV())) {
        delete truthValue;
    }
    truthValue = (TruthValue*) &(TruthValue::DEFAULT_TV());
    if (!tv.isNullTv() && (&tv != &(TruthValue::DEFAULT_TV()))) {
        truthValue = tv.clone();
    }
#else 
    if (truthValue != NULL && &tv != truthValue) {
        delete truthValue;
    }
#if 1
    // just like it was before    
    truthValue = tv.clone();
#else    
    if (!tv.isNullTv()) {
        truthValue = tv.clone();
    } else {
        truthValue = TruthValue::DEFAULT_TV().clone();
    }
#endif    
#endif     
}

//float Atom::getImportance() {

//    return ShortFloatOps::getValue(&importance);
//}

void Atom::setAttentionValue(const AttentionValue& new_av) throw (RuntimeException) {
    //// if the value is out of bounds, it is set to either the upper or lower bound
    //if (value > 1) value = 1;
    //if (value < 0) value = 0;

    int oldBin;
    if (atomTable != NULL) {
        // gets current bin
        oldBin = AtomTable::importanceBin(attentionValue->getSTI());
    }

    //// this MUST come before updateImportanceIndex
    //rawSetImportance(value);
    if(attentionValue != NULL && &new_av != attentionValue)
        delete attentionValue;

    attentionValue = new_av.clone();

    if (atomTable != NULL) {
        // gets new bin
        int newBin = AtomTable::importanceBin(attentionValue->getSTI());

        // if the atom importance has changed its bin, 
        // updates the importance index
        if (oldBin != newBin) {
#ifdef USE_MIND_DB_PROXY
            /* THIS WAS ONLY NEEDED WHEN ATOM HAD NO ATTRIBUTE WITH ITS TABLE ID.
               AtomTable *table;
            // cycles trhough all the tables searching for the Atom.
            // This implementation priviledges the most commom case DEFAULT
            // We did not use an identifier for the table the atom is inserted in 
            // because of space constraints.
            AtomTableList t;
            for (t = DEFAULT; t < ATOM_TABLE_LIST_SIZE; t++){
            table = MindDBProxy::getInstance()->getAtomTable(t);
            if (table->updateImportanceIndex(this, oldBin)){
            break;
            }
            }
            if (t == ATOM_TABLE_LIST_SIZE){
            throw RuntimeException(TRACE_INFO, "failed to locate atom in importance bin");
            }
            StatisticsMonitor::getInstance()->atomChangeImportanceBin(type, oldBin, newBin);
            */
#endif

            if (!atomTable->updateImportanceIndex(this, oldBin)) {
                throw RuntimeException(TRACE_INFO, "failed to locate atom in importance bin");
            }
            StatisticsMonitor::getInstance()->atomChangeImportanceBin(type, oldBin, newBin);
        }
    }
}

//void Atom::rawSetImportance(float value) {

//    // if the value is out of bounds, it is set to either the upper or lower bound
//    if (value > 1) value = 1;
//    if (value < 0) value = 0;

//    ShortFloatOps::setValue(&importance, value);
//}

void Atom::setOutgoingSet(const std::vector<Handle>& outgoingVector)  throw (RuntimeException) {
    //printf("Atom::setOutgoingSet\n");
    if (atomTable != NULL){
        throw RuntimeException(TRACE_INFO, "Cannot change the OutgoingSet of an atom already inserted into an AtomTable\n");
    }
#ifdef USE_STD_VECTOR_FOR_OUTGOING
    outgoing = outgoingVector;
    // if the link is unordered, it will be normalized by sorting the elements in the outgoing list.
    if (ClassServer::isAssignableFrom(UNORDERED_LINK, type)) {
        std::sort(outgoing.begin(), outgoing.end(), CoreUtils::HandleComparison());
    }
#else
    if (outgoing) {
        free(outgoing);
        outgoing = NULL;
    }
    arity = outgoingVector.size(); 
    if (arity > 0) { 
        outgoing = (Handle*) malloc(sizeof(Handle)*arity);
        for (int i = 0; i < arity; i++) {
            outgoing[i] = outgoingVector[i];
        }
    } 
    if (ClassServer::isAssignableFrom(UNORDERED_LINK, type)) {
        qsort(outgoing, arity, sizeof(Handle), CoreUtils::handleCompare);
    }
#endif
}

void Atom::addOutgoingAtom(Handle h) {
#ifdef USE_STD_VECTOR_FOR_OUTGOING
    outgoing.push_back(h);
#else 
    outgoing = (Handle*) realloc(outgoing, (arity+1)*sizeof(Handle));
    outgoing[arity++] = h;
#endif    
}

Atom * Atom::getOutgoingAtom(int position) const throw (RuntimeException)
{
    // checks for a valid position
    if ((position < getArity()) && (position >= 0)) {
        return TLB::getAtom(outgoing[position]);
    } else {
        throw RuntimeException(TRACE_INFO, "invalid outgoing set index %d", position);
    }
}

void Atom::addIncomingHandle(Handle handle) {

    // creates a new entry with handle
    HandleEntry* entry = new HandleEntry(handle);
    // entry is placed in the first position of the incoming set
    entry->next = incoming;
    incoming = entry;
}

void Atom::removeIncomingHandle(Handle handle) throw (RuntimeException) {

	//printf("Entering Atom::removeIncomingHandle(): handle:\n%s\nincoming:\n%s\n", TLB::getAtom(handle)->toShortString().c_str(), incoming->toString().c_str());
    HandleEntry* current = incoming;
    // checks if incoming set is empty
    if (incoming == NULL) {
        throw RuntimeException(TRACE_INFO, "unable to extract incoming element from empty set");
    }

    // checks if the handle to be removed is the first one
    if (incoming->handle == handle) {
        incoming = incoming->next;
        current->next = NULL;
        delete current;
    } else {
        if (current->next == NULL) {
            throw RuntimeException(TRACE_INFO, "unable to extract incoming element");
        }
        // scans the list looking for the desired handle
        while (current->next->handle != handle) {
            current = current->next;
            if (current->next == NULL) {
                throw RuntimeException(TRACE_INFO, "unable to extract incoming element");
            }
        }
        // deletes entry when the handle is found
        HandleEntry* foundit = current->next;
        current->next = foundit->next;
        foundit->next = NULL;
        delete foundit;
    }
	//printf("Exiting Atom::removeIncomingHandle(): incoming:\n%s\n", incoming->toString().c_str());
}

void Atom::setNext(int index, Handle handle) {
    //printf("Setting next of index %p, handle=%p\n", index, handle);
    //printf("PREDICATE_INDEX = %p!\n", PREDICATE_INDEX);
    //printf("TARGET_TYPE_INDEX = %p!\n", TARGET_TYPE_INDEX);

    // the index parameter contains the index of the list that must be
    // traversed by the method. Optionally, the index parameter may bring the
    // index in the target types linked-list to be traversed, in which case it
    // has its value OR-ed (binary) to the TARGET_TYPE_INDEX flag
    if (index & TARGET_TYPE_INDEX) {
        index &= 0x0000ffff;
        int targetIndex = locateTargetIndexTypes(index);
        if (targetIndex != -1) {
            targetTypeIndex[targetIndex] = handle;
        } else {
            throw RuntimeException(TRACE_INFO, "could not find target type index");
        }
    } else if (index & PREDICATE_INDEX) {
        //cprintf(DEBUG, "setNext(%p,%p) => Predicate index!\n", index, handle);
        if (predicateIndexInfo == NULL) {
            throw RuntimeException(TRACE_INFO, "no info about predicate indices");
        }
        index &= ~PREDICATE_INDEX;
        unsigned long indexMask = (1UL<<index);
        //cprintf(DEBUG,"Index = %p (mask = %p)\n", index, indexMask);
        if (!(predicateIndexInfo->predicateIndexMask & indexMask)) {
            throw RuntimeException(TRACE_INFO, "could not find predicate index");
        }
        setNextHandleInPredicateIndex(index, handle);
    } else {
        indices[index] = handle;
    }
}

Handle* Atom::getTargetTypeIndex() const{
    return targetTypeIndex;
}

void Atom::setNextTargetTypeIndex(Handle* handles) {
    targetTypeIndex = handles;
}

Handle Atom::next(int index) {
    //printf("Getting next of index %p\n", index);
    //printf("PREDICATE_INDEX = %p!\n", PREDICATE_INDEX);
    //printf("TARGET_TYPE_INDEX = %p!\n", TARGET_TYPE_INDEX);

    // the index parameter contains the index of the list that must be
    // traversed by the method. Optionally, the index parameter may bring the
    // index in the target types linked-list to be traversed, in which case it
    // has its value OR-ed (binary) to the TARGET_TYPE_INDEX flag.
    if (index & TARGET_TYPE_INDEX) {
        index &= ~TARGET_TYPE_INDEX;
        int targetIndex = locateTargetIndexTypes(index);
        if (targetIndex != -1) {
            return targetTypeIndex[targetIndex];
        } else {
            throw RuntimeException(TRACE_INFO, "could not find target type index");
        }
    } else if (index & PREDICATE_INDEX) {
        //cprintf(DEBUG,"next(%p) => Predicate index!\n", index);
        if (predicateIndexInfo == NULL) {
            throw RuntimeException(TRACE_INFO, "no info about predicate indices");
        }
        index &= ~PREDICATE_INDEX;
        //cprintf(DEBUG,"Index = %p (mask = %p)\n", index, (1UL<<index));
        if (predicateIndexInfo->predicateIndexMask & (1UL<<index)) {
            Handle result = getNextHandleInPredicateIndex(index);
            //cprintf(DEBUG,"next = %p\n", result);
            return result;
        } else {
            throw RuntimeException(TRACE_INFO, "could not find predicate index");
        }
    } else {
        return indices[index];
    }
}

Handle Atom::getNextHandleInPredicateIndex(int index) const{
    // NOTE: Here, we know that index exists already.
    unsigned long maskOfMask = ~(0xFFFFFFFFUL << index);
    int pos = bitcount(predicateIndexInfo->predicateIndexMask & maskOfMask);
    //cprintf(DEBUG,"getNextHandleInPredicateIndex(%d) => (maskOfMask = %p, predicateIndexMask = %p) => pos = %d\n", index, maskOfMask, predicateIndexInfo->predicateIndexMask, pos);
    return predicateIndexInfo->predicateIndex[pos];
}

void Atom::setNextHandleInPredicateIndex(int index, Handle nextHandle) {
    // NOTE: Here, we know that index exists already.
    unsigned long maskOfMask = ~(0xFFFFFFFFUL << index);
    int pos = bitcount(predicateIndexInfo->predicateIndexMask & maskOfMask);
    //cprintf(DEBUG,"setNextHandleInPredicateIndex(%d,%p) => (maskOfMask = %p, predicateIndexMask = %p) => pos = %d\n", index, nextHandle, maskOfMask, predicateIndexInfo->predicateIndexMask, pos);
    predicateIndexInfo->predicateIndex[pos] = nextHandle;
}

void Atom::addNextPredicateIndex(int index, Handle nextHandle) {
    //    printf("Atom(%p)::addNextPredicateIndex(%d, %p)\n", this, index, nextHandle);
    // NOTE: Here, we know that index is not added yet.
    if (predicateIndexInfo == NULL) {
        // first index
        predicateIndexInfo = new PredicateIndexStruct();
        predicateIndexInfo->predicateIndexMask = (1UL << index);
        predicateIndexInfo->predicateIndex = new Handle[1];
        predicateIndexInfo->predicateIndex[0] = nextHandle;
    } else {
        unsigned long mask = predicateIndexInfo->predicateIndexMask;
        Handle* oldPredicateIndex = predicateIndexInfo->predicateIndex;
        // Figure out the position of the new predicate index
        unsigned long maskOfMask = ~(0xFFFFFFFFUL << index);
        int pos = bitcount(mask & maskOfMask);
        // Gets the new size and increment it by one
        int size = bitcount(mask)+1;
        // allocates a new array for indices and set its positions
        Handle* newPredicateIndex = new Handle[size];
        for (int i = 0; i < size; i++) {
            if (i < pos) {
                newPredicateIndex[i] = oldPredicateIndex[i];
            } else if (i > pos) {
                newPredicateIndex[i] = oldPredicateIndex[i-1];
            } else {
                newPredicateIndex[i] = nextHandle;
            }   
        }
        predicateIndexInfo->predicateIndex = newPredicateIndex;
        predicateIndexInfo->predicateIndexMask |= (1UL << index);
    }
}

void Atom::merge(Atom* other) throw (InconsistenceException){

    if (!equals(other)){
        throw InconsistenceException(TRACE_INFO, "Different atoms cannot be merged");
    }

    //merges the incoming set and updated the outgoingSets
    HandleEntry *inc = other->getIncomingSet();
    Handle thisHandle = TLB::getHandle(this);
    // ricbit changed the second "this" for "other"
    Handle otherHandle = TLB::getHandle(other);
    //Handle otherHandle = TLB::getHandle(this);
    while (inc != NULL){
        addIncomingHandle(inc->handle);

        Atom *incAtom = TLB::getAtom(inc->handle);
        std::vector<Handle> outgoingSet = incAtom->getOutgoingSet();
        for (int i = 0; i < incAtom->getArity(); i++){
            if (eqHandle()(outgoingSet[i], otherHandle)){
                outgoingSet[i] = thisHandle;
            }
        }
        // Although we have direct access to outgoing here, we need to call setOutgoingSet anyway, 
        // since special handling may be need by subclasses (e.g, sorting of the outgoing, if it's an unordered link)
        incAtom->setOutgoingSet(outgoingSet); 
        inc = inc->next;
    }


    //setImportance((getImportance() + other->getImportance()) / 2);
    //setHeat((getHeat() + other->getHeat()) / 2);

#ifdef USE_SHARED_DEFAULT_TV
    // TruthValue::merge() method always return a new TV object
    TruthValue* mergedTv = truthValue->merge(other->getTruthValue()); 
    if (truthValue != &(TruthValue::DEFAULT_TV())) {
        delete(truthValue);
    }
    if (mergedTv == &(TruthValue::DEFAULT_TV())) {
        delete(mergedTv);
        truthValue = (TruthValue*) &(TruthValue::DEFAULT_TV());
    } else {
        truthValue = mergedTv;
    }
#else
    TruthValue* mergedTv = truthValue->merge(other->getTruthValue()); // always return a new TV object
    delete(truthValue);
    truthValue = mergedTv;
#endif    

    //cprintf(DEBUG, ">> This atom's truth values after merge: %f %f %f\n", truthValue->getMean(), truthValue->getConfidence(), truthValue->getCount()); 
}

bool Atom::isMarkedForRemoval() const{
	//printf("Atom::isMarkedForRemoval(): %p\n", this);
    return flags & MARKED_FOR_REMOVAL;
}

int Atom::locateTargetIndexTypes(Type key) const{

    if (getArity() == 0) return -1;

    set<unsigned int> checkedTypes;
    int j = 0;
    // for each type in the target types array, it checks if it has already
    // been found so that repeated types are not considered. If a type is equal
    // to the parameter passed, its position in the array is returned
    for (int i = 0; i < getArity(); i++) {
        Type type = TLB::getAtom(outgoing[i])->getType();
        if (checkedTypes.find(type) == checkedTypes.end()) {
            if (type == key) {
                return j;
            }
            j++;
            checkedTypes.insert(type);
        }
    }

    // if the parameter type is not found in the array returns -1
    return -1;
}

Type* Atom::buildTargetIndexTypes(int *size) {

    *size = 0;
    if (getArity() == 0) {
        return NULL;
    }

    // types array has size arity
    Type* types = new Type[getArity()];

    // keeps a boolean for every possible type
    std::set<unsigned int> checkedTypes;

    // fills the types array with new entries only, ignoring duplicated types
    for (int i = 0; i < getArity(); i++) {
        Type type = TLB::getAtom(outgoing[i])->getType();
        if (checkedTypes.find(type) == checkedTypes.end()) {
            types[(*size)++] = type;
            checkedTypes.insert(type);
        }
    }

    return types;
}

int Atom::getTargetTypeIndexSize() const{

    std::set<unsigned int> checkedTypes;
    int j = 0;
    // for each type in the target types array, it checks if it has already
    // been found so that repeated types are not considered in the counting 
    for (int i = 0; i < getArity(); i++) {
        Type type = TLB::getAtom(outgoing[i])->getType();
        if (checkedTypes.find(type) == checkedTypes.end()) {
            j++;
            checkedTypes.insert(type);
        }
    }

    // returns the number of different target types
    return j;
}

bool Atom::hasPredicateIndexInfo() {
    return (predicateIndexInfo != NULL);
}

int* Atom::buildPredicateIndices(int *size) const{
    unsigned long mask = predicateIndexInfo->predicateIndexMask;
    *size = bitcount(mask);
    int* result = new int[*size ];
    int pos = 0;
    int predicateIndex = 0;
    while (mask) {
        if (mask & 1) {
            result[pos++] = predicateIndex;
        }
        predicateIndex++;    
        mask >>= 1;    
    }
    return result;
}

bool Atom::getFlag(int flag) const{
    return flags & flag;
}

void Atom::setFlag(int flag, bool value) {
    if (value) {
        flags |= flag;
    } else {
        flags &= ~(flag);
    }
}

void Atom::unsetRemovalFlag() {
    flags &= ~MARKED_FOR_REMOVAL;
}

void Atom::markForRemoval() {
    flags |= MARKED_FOR_REMOVAL;
}


void Atom::setAtomTable(AtomTable *tb){
    atomTable = tb;
}

AtomTable *Atom::getAtomTable() const{
    return(atomTable);
}

bool Atom::equals(Atom* other){
    bool equal = ((type == other->type) &&
            (getArity() == other->getArity()));

    if (equal){
        for (int i = 0; i < getArity(); i++){
            if (outgoing[i] != other->outgoing[i]){
                equal = false;
                break;
            }
        }
    }

    return(equal);
}

int Atom::hashCode()
{
    long result = type + (getArity()<<8);

    for (int i = 0; i < getArity(); i++){
        result = result  ^ (((long) outgoing[i])<<i);
    }
    return (int) result;
}

HandleEntry *Atom::getNeighbors(bool fanin, bool fanout, Type desiredLinkType, bool subClasses) const{

    HandleEntry *answer = NULL;
    Handle me = TLB::getHandle(this);

    for (HandleEntry *h = getIncomingSet(); h != NULL; h = h ->next) {
        Link *link = (Link *) TLB::getAtom(h->handle);
        Type linkType = link->getType();
        //printf("linkType = %d desiredLinkType = %d\n", linkType, desiredLinkType);
        if ((linkType == desiredLinkType) || (subClasses && ClassServer::isAssignableFrom(desiredLinkType, linkType))) {
            int linkArity = link->getArity();
            for (int i = 0; i < linkArity; i++) {
                Handle handle = link->getOutgoingSet()[i];
                if (handle == me) continue;
                if (!fanout && link->isSource(me)) continue;
                if (!fanin && link->isTarget(me)) continue;
                HandleEntry *n = new HandleEntry(handle);
                n->next = answer;
                answer = n;
            }
        }
    }

    return answer;
}


/*
   void Atom::beginRead() {
   pthread_mutex_lock(&lock);
   while (flags & WRITE_MUTEX) {
   pthread_cond_wait(&cond, &lock);
   }
   readSemaphore++;
   pthread_mutex_unlock(&lock);
   }

   void Atom::endRead() {
   pthread_mutex_lock(&lock);
   readSemaphore--;
   if (readSemaphore == 0) {
//cond_signal();
pthread_cond_broadcast(&cond);
}
pthread_mutex_unlock(&lock);
}

void Atom::beginWrite() {
pthread_mutex_lock(&lock);
while ((readSemaphore > 0) && (flags & WRITE_MUTEX)) {
pthread_cond_wait(&cond, &lock);
}
flags |= WRITE_MUTEX;
pthread_mutex_unlock(&lock);
}

void Atom::endWrite() {
pthread_mutex_lock(&lock);
flags &= ~WRITE_MUTEX;
pthread_cond_broadcast(&cond);
pthread_mutex_unlock(&lock);
}
*/
