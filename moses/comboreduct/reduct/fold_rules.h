/*
 *moses/comboreduct/reduct/fold_rules.h
 */

#ifndef _REDUCT_FOLD_RULES_H
#define _REDUCT_FOLD_RULES_H

#include "reduct.h"

namespace moses3 { namespace reduct {

//fold unrolling:
//    foldl(f v list(a b c)) = f(f(f(v a) b) c)
//    foldr(f v list(a b c)) = f(a f(b f(c v)))
struct fold_unrolling : public crule<fold_unrolling> {
    fold_unrolling() : crule<fold_unrolling>::crule("fold_unrolling") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const;
};

} // ~namespace reduct
} // ~namespace moses3

#endif
