/*
 * opencog/embodiment/Control/LoggerFactory.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef LOGGERFACTORY_H
#define LOGGERFACTORY_H

#include <string>
#include <opencog/util/Logger.h>
#include "EmbodimentConfig.h"

namespace opencog {
namespace control {

class LoggerFactory
{

private:

    LoggerFactory();

public:

    static opencog::Logger getLogger(const std::string &id);

}; // class
} } // namespace

#endif
