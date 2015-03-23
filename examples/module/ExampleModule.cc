/*
 * examples/modules/ExampleModule.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * Copyright (C) 2014 Linas Vepstas
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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


#include <opencog/util/Logger.h>

#include "ExampleModule.h"

using namespace opencog;

// load/unload functions for the Module interface
DECLARE_MODULE(ExampleModule)

ExampleModule::ExampleModule(CogServer& cs) : Module(cs)
{
    logger().info("[ExampleModule] constructor");
}

ExampleModule::~ExampleModule()
{
    logger().info("[ExampleModule] destructor");
}

void ExampleModule::init()
{
    logger().info("[ExampleModule] init");
}

