/*
 * MinerSCM.cc
 *
 * Copyright (C) 2018 OpenCog Foundation
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com> 
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

#ifdef HAVE_GUILE

#include <cmath>

#include <opencog/util/Logger.h>
#include <opencog/guile/SchemeModule.h>
#include <opencog/atoms/core/NumberNode.h>

#include "MinerUtils.h"
#include "Surprisingness.h"

namespace opencog {

class MinerSCM : public ModuleWrap
{
protected:
	virtual void init();

	/**
	 * Given a pattern, a texts concept and a minimum support, return
	 * all shallow abstractions reaching the minimum support,
	 * pre-wrapped in Lists ready to be applied to the pattern.
	 *
	 * For instance, given
	 *
	 * pattern = (Lambda X Y (Inheritance X Y))
	 * texts  = { (Inheritance A B),
	 *            (Inheritance A C),
	 *            (Inheritance D D),
	 *            (Inheritance E E) }
	 * ms = (Number 2)
	 *
	 * returns
	 *
	 * Set
	 *   List A Y
	 *   List Y Y
	 */
	Handle do_shallow_abstract(Handle pattern, Handle texts, Handle ms);

	/**
	 * Given a pattern, a texts concept and a minimum support, return
	 * all shallow specializations reaching the minimum support.
	 *
	 * For instance, given
	 *
	 * pattern = (Lambda X Y (Inheritance X Y))
	 * texts  = { (Inheritance A B),
	 *            (Inheritance A C),
	 *            (Inheritance D D),
	 *            (Inheritance E E) }
	 * ms = (Number 2)
	 *
	 * returns
	 *
	 * (Set
	 *   (Lambda Y (Inheritance A Y))
	 *   (Lambda Y (Inheritance Y Y)))
	 */
	Handle do_shallow_specialize(Handle pattern, Handle texts, Handle ms);

	/**
	 * Given a pattern, a texts concept and a minimum support, return
	 * true iff the pattern has enough support.
	 */
	bool do_enough_support(Handle pattern, Handle texts, Handle ms);

	/**
	 * Construct the conjunction of 2 patterns. If cnjtion is a
	 * conjunction, then expand it with pattern. It is assumed that
	 * pattern cannot be a conjunction itself.
	 */
	Handle do_expand_conjunction(Handle cnjtion, Handle pattern,
	                             Handle texts, Handle ms);

	/**
	 * Calculate the I-Surprisingness of the pattern (and its
	 * partitions) with respect to texts.
	 *
	 * do_isurp_old: Shujing I-Surprisingness
	 * do_nisurp_old: Shujing normalized I-Surprisingness
	 * do_isurp: I-Surprisingness
	 * do_nisurp: normalized I-Surprisingness
	 */
	double do_isurp_old(Handle pattern, Handle texts);
	double do_nisurp_old(Handle pattern, Handle texts);
	double do_isurp(Handle pattern, Handle texts);
	double do_nisurp(Handle pattern, Handle texts);

public:
	MinerSCM();
};

} /*end of namespace opencog*/

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemePrimitive.h>

#include <opencog/learning/miner/Miner.h>

using namespace opencog;

MinerSCM::MinerSCM() : ModuleWrap("opencog miner") {}

/// This is called while (opencog miner) is the current
/// module.  Thus, all the definitions below happen in that module.
void MinerSCM::init(void)
{
	define_scheme_primitive("cog-shallow-abstract",
		&MinerSCM::do_shallow_abstract, this, "miner");

	define_scheme_primitive("cog-shallow-specialize",
		&MinerSCM::do_shallow_specialize, this, "miner");

	define_scheme_primitive("cog-enough-support?",
		&MinerSCM::do_enough_support, this, "miner");

	define_scheme_primitive("cog-expand-conjunction",
		&MinerSCM::do_expand_conjunction, this, "miner");

	define_scheme_primitive("cog-isurp-old",
		&MinerSCM::do_isurp_old, this, "miner");

	define_scheme_primitive("cog-nisurp-old",
		&MinerSCM::do_nisurp_old, this, "miner");

	define_scheme_primitive("cog-isurp",
		&MinerSCM::do_isurp, this, "miner");

	define_scheme_primitive("cog-nisurp",
		&MinerSCM::do_nisurp, this, "miner");
}

Handle MinerSCM::do_shallow_abstract(Handle pattern,
                                     Handle texts,
                                     Handle ms)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("cog-shallow-abstract");

	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	// Fetch the minimum support
	unsigned ms_uint = MinerUtils::get_ms(ms);

	// Generate all shallow abstractions
	HandleSetSeq shabs_per_var =
		MinerUtils::shallow_abstract(pattern, texts_seq, ms_uint);

	// Turn that sequence of handle sets into a set of ready to be
	// applied shallow abstractions
	const Variables& vars = MinerUtils::get_variables(pattern);
	HandleSet sa_lists;
	unsigned vari = 0;         // Index of the variable
	for (const HandleSet& shabs : shabs_per_var) {
		for (const Handle& sa : shabs) {
			HandleSeq sa_list = vars.varseq;
			sa_list[vari] = sa;
			sa_lists.insert(sa_list.size() == 1 ? sa_list[0]
			                // Only Wrap in a list if arity is greater
			                // than one
			                : as->add_link(LIST_LINK, sa_list));
		}
		vari++;
	}

	return as->add_link(SET_LINK, HandleSeq(sa_lists.begin(), sa_lists.end()));
}

Handle MinerSCM::do_shallow_specialize(Handle pattern,
                                       Handle texts,
                                       Handle ms)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("cog-shallow-specialize");

	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	// Fetch the minimum support
	unsigned ms_uint = MinerUtils::get_ms(ms);

	// Generate all shallow specializations
	HandleSet shaspes = MinerUtils::shallow_specialize(pattern, texts_seq, ms_uint);

	return as->add_link(SET_LINK, HandleSeq(shaspes.begin(), shaspes.end()));
}

bool MinerSCM::do_enough_support(Handle pattern, Handle texts, Handle ms)
{
	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	// Fetch the minimum support
	unsigned ms_uint = MinerUtils::get_ms(ms);

	return MinerUtils::enough_support(pattern, texts_seq, ms_uint);
}

Handle MinerSCM::do_expand_conjunction(Handle cnjtion, Handle pattern,
                                       Handle texts, Handle ms)
{
	AtomSpace *as = SchemeSmob::ss_get_env_as("cog-expand-conjunction");

	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	// Fetch the minimum support
	unsigned ms_uint = MinerUtils::get_ms(ms);

	HandleSet results = MinerUtils::expand_conjunction(cnjtion, pattern,
	                                                   texts_seq,
	                                                   ms_uint);
	return as->add_link(SET_LINK, HandleSeq(results.begin(), results.end()));
}

double MinerSCM::do_isurp_old(Handle pattern, Handle texts)
{
	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	return Surprisingness::isurp_old(pattern, texts_seq, false);
}

double MinerSCM::do_nisurp_old(Handle pattern, Handle texts)
{
	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	return Surprisingness::isurp_old(pattern, texts_seq, true);
}

double MinerSCM::do_isurp(Handle pattern, Handle texts)
{
	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	return Surprisingness::isurp(pattern, texts_seq, false);
}

double MinerSCM::do_nisurp(Handle pattern, Handle texts)
{
	// Fetch all texts
	HandleSeq texts_seq = MinerUtils::get_texts(texts);

	return Surprisingness::isurp(pattern, texts_seq, true);
}

extern "C" {
void opencog_miner_init(void);
};

void opencog_miner_init(void)
{
    static MinerSCM miner;
    miner.module_init();
}

#endif // HAVE_GUILE
