/*
 * opencog/server/DimEmbedModule.h
 *
 * Copyright (C) 2010 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Written by David Crane <dncrane@gmail.com>
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

#include "DimEmbedModule.h"
#include <opencog/util/Logger.h>

using namespace opencog;

DECLARE_MODULE(DimEmbedModule);


DimEmbedModule::DimEmbedModule() {
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerAgent(DimensionalEmbedding::info().id, &dimEmbedFactory);
    cogserver.createAgent(DimensionalEmbedding::info().id, true);
}

DimEmbedModule::~DimEmbedModule() {
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.unregisterAgent(DimensionalEmbedding::info().id);
}

void DimEmbedModule::init() {
    logger().info("DimEmbed init");
    }
