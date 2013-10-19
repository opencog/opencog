/*
 * opencog/rest/JsonUtil.h
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * Copyright (C) 2010 by Joel Pitt
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#include <opencog/atomspace/TruthValue.h>

#include <opencog/web/json_spirit/json_spirit.h>

using namespace json_spirit;

namespace opencog {

bool assertJSONTVCorrect(std::string expected, std::string actual,
        std::ostringstream& _output );
bool assertJsonMapContains(const Object& o, std::vector<std::string> keys,
        std::ostringstream& _output );
TruthValuePtr JSONToTV(const Value& v, std::ostringstream& _output );

} // namespace
