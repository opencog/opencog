/*
 * @file opencog/modules/ExecModule.cc
 * @author Amen Belayneh <amenbelayneh@gmail.com> June 2015
 *
 * Copyright (C) 2015 OpenCog Foundation
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

#include "ExecModule.h"

using namespace opencog;

DECLARE_MODULE(ExecModule);

ExecModule::ExecModule(CogServer& cs) : Module(cs)
{
}

ExecModule::~ExecModule()
{
    delete _exec;
}

void ExecModule::init()
{
    _exec = new ExecSCM();
    _exec->module_init();
}
