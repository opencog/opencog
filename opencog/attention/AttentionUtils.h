#ifndef OPENCOG_ATTENTION_UTILS_H
#define OPENCOG_ATTENTION_UTILS_H

#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

/*
 * Remove HebbianLink type atoms from a HandleSeq;
 *
 * @param sources A reference to HandleSeq
 *
 * @return void
 *
 */
void removeHebbianLinks(HandleSeq& sources);
/** @}*/
} // namespace

#endif
