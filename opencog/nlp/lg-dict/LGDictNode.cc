/*
 * opencog/nlp/lg-dict/LGDictNode.cc
 *
 * Copyright (C) 2017 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <mutex>

#include <link-grammar/link-includes.h>
#include <opencog/atoms/proto/NameServer.h>
#include <opencog/util/Logger.h>
#include <opencog/nlp/types/atom_types.h>

#include "LGDictNode.h"

using namespace opencog;

// ------------------------------------------------------
// Convert LG errors to opencog log messges
void error_handler(lg_errinfo *ei, void *data);
void error_handler(lg_errinfo *ei, void *data)
{
	if (lg_Fatal == ei->severity or lg_Error == ei->severity)
		logger().error("%s", ei->text);
	else if (lg_Warn == ei->severity)
		logger().warn("%s", ei->text);
	else
		logger().info("%s", ei->text);
}

// ------------------------------------------------------

LgDictNode::LgDictNode(const std::string& name)
	: Node(LG_DICT_NODE, name), _dict(nullptr)
{
}

LgDictNode::LgDictNode(const Node& n)
	: Node(n), _dict(nullptr)
{
	// Type must be as expected
	Type tdict = n.get_type();
	if (not nameserver().isA(tdict, LG_DICT_NODE))
	{
		const std::string& tname = nameserver().getTypeName(tdict);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LgDictNode, got %s", tname.c_str());
	}
}

LgDictNode::~LgDictNode()
{
	if (_dict)
		dictionary_delete(_dict);

	_dict = nullptr;
}

/// Get the dictionary associated with the node.  This performs a
/// delayed open, because we don't really want the open to happen
/// in the constructor (since the constructor might run mmultiple
/// times!?)
Dictionary LgDictNode::get_dictionary()
{
	if (_dict) return _dict;

	lg_error_set_handler(error_handler, nullptr);
	const char * lang = get_name().c_str();

	// Link grammar dictionary creation is NOT thread-safe.
	static std::mutex _global_mtx;
	std::lock_guard<std::mutex> lck(_global_mtx);

	// Check again, this time under the lock.
	if (_dict) return _dict;

	_dict = dictionary_create_lang(lang);
	return _dict;
}

// ------------------------------------------------------
// Factory stuff.

Handle LgDictNode::factory(const Handle& base)
{
	if (LgDictNodeCast(base)) return base;
	Handle h(createLgDictNode(base->get_name()));
	return h;
}

/* This runs when the shared lib is loaded. */
static __attribute__ ((constructor)) void init(void)
{
   classserver().addFactory(LG_DICT_NODE, &LgDictNode::factory);
}

// ------------------------------------------------------

/* This allows guile to load this shared library */
extern "C" {
	void opencog_nlp_lgparse_init(void) {}
};
