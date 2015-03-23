/*
 * opencog/learning/moses/eda/logging.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _EDA_LOGGING_H
#define _EDA_LOGGING_H

#include <algorithm>
#include <iostream>
#include <sstream>
#include <opencog/util/Logger.h>

#include "../representation/field_set.h"

namespace opencog {
namespace moses {

    struct cout_log_best_and_gen
    {
        template<typename It>
        void operator()(It from, It to, const field_set& fs, int gen) const
        {
            if (!logger().isDebugEnabled())
                return;

            if (from == to)
                return;

            It best = std::max_element(from, to);
            logger().debug("Generation: %d", gen);
            std::stringstream ss;
            ss << "Best instance: " << best->second << " "
               << fs.to_string(best->first);
            logger().debug(ss.str());
        }
    };

} // ~namespace moses
} // ~namespace opencog

#endif
