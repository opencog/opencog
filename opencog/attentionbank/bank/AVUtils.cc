/*
 * opencog/attentionbank/bank/AVUtils.cc
 *
 * Copyright (C) 2017 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
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

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/attentionbank/bank/AVUtils.h>

using namespace opencog;

static const Handle& attn_key(void)
{
	static Handle ak(createNode(PREDICATE_NODE, "*-AttentionValueKey-*"));
	return ak;
}

AttentionValuePtr opencog::get_av(const Handle& h)
{
    auto pr = h->getValue(attn_key());
    if (nullptr == pr) return AttentionValue::DEFAULT_AV();
    return AttentionValueCast(pr);
}

void opencog::set_av(AtomSpace* as, const Handle& h, const AttentionValuePtr& av)
{
    as->set_value(h, attn_key(), ValueCast(av));
}
