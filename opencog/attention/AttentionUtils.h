#ifndef ATTENTIONUTILS_H
#define ATTENTIONUTILS_H

#include <opencog/atomspace/AtomSpace.h>

namespace opencog{

/*
 * Remove HebbianLink type atoms from a HandleSeq;
 *
 * @param sources A reference to HandleSeq
 *
 * @return void
 *
 */
void removeHebbianLinks(HandleSeq& sources);

}
#endif
