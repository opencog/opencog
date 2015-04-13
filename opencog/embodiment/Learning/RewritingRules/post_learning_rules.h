/*
 * opencog/embodiment/Learning/RewritingRules/post_learning_rules.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _POST_LEARNING_RULES_H
#define _POST_LEARNING_RULES_H

#include <opencog/comboreduct/reduct/reduct.h>

namespace opencog { namespace reduct {

//add a drop action in front of a grab action
struct post_learning_drop_before_grab
            : public crule<post_learning_drop_before_grab> {
    post_learning_drop_before_grab() : crule<post_learning_drop_before_grab>::crule("post_learning_drop_before_grab") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

//add action_success at child of any empty and_seq
struct post_learning_empty_and_seq
            : public crule<post_learning_empty_and_seq> {
    post_learning_empty_and_seq() : crule<post_learning_empty_and_seq>::crule("post_learning_empty_and_seq") {}
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
};

} // ~namespace reduct
} // ~namespace opencog

#endif
