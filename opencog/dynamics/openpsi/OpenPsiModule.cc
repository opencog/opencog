/*
 * @file opencog/dynamics/openpsi/OpenPsiModule.cc
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

#include "OpenPsiModule.h"

#include <opencog/server/CogServer.h>

using namespace opencog;

DECLARE_MODULE(OpenPsiModule);

/**
 * The constructor for OpenPsiModule.
 *
 * @param cs   A cogserver
 */
OpenPsiModule::OpenPsiModule(CogServer& cs) : Module(cs)
{
    logger().info("[OpenPsiModule] Entering constructor");
    _cogserver.registerAgent(PsiDemandUpdaterAgent::info().id,
                            &demandUpdaterFactory);
    _cogserver.registerAgent(PsiActionSelectionAgent::info().id,
                            &actionSelectionFactory);
    _cogserver.registerAgent(PsiModulatorUpdaterAgent::info().id,
                            &modulatorUpdaterFactory);
    _cogserver.registerAgent(PsiFeelingUpdaterAgent::info().id,
                            &feelingUpdaterFactory);
}

OpenPsiModule::~OpenPsiModule()
{
    logger().info("[OpenPsiModule] Entering destructor");
    _cogserver.unregisterAgent(PsiDemandUpdaterAgent::info().id);
    _cogserver.unregisterAgent(PsiActionSelectionAgent::info().id);
    _cogserver.unregisterAgent(PsiModulatorUpdaterAgent::info().id);
    _cogserver.unregisterAgent(PsiFeelingUpdaterAgent::info().id);
}


void OpenPsiModule::init()
{
    logger().info("[OpenPsiModule] Initializing OpenPsiModule.");
}
