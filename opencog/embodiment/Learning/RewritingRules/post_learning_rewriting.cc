/*
 * opencog/embodiment/Learning/RewritingRules/post_learning_rewriting.cc
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

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/reduct/meta_rules.h>

#include "post_learning_rewriting.h"
#include "post_learning_rules.h"

namespace opencog { namespace reduct {

const rule& post_learning_rewriting()
{
    static sequential r =
        sequential(downwards(post_learning_drop_before_grab()),
                   downwards(post_learning_empty_and_seq()));

    return r;
}

} // ~namespace reduct
} // ~namespace opencog

