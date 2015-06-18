/*
 * @file opencog/dynamics/openpsi/OpenPsiModule.h
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

#ifndef _OPENCOG_DYNAMICS_OPENPSI_H
#define _OPENCOG_DYNAMICS_OPENPSI_H

#include <opencog/server/Module.h>
#include <opencog/dynamics/openpsi/PsiDemandUpdaterAgent.h>
#include <opencog/dynamics/openpsi/PsiActionSelectionAgent.h>
#include <opencog/dynamics/openpsi/PsiModulatorUpdaterAgent.h>

namespace opencog
{

/**
 * OpenCog Module for OpnePsi
*/

class OpenPsiModule : public Module
{
private:
    Factory<PsiDemandUpdaterAgent, Agent> demandUpdaterFactory;
    Factory<PsiActionSelectionAgent, Agent> actionSelectionFactory;
    Factory<PsiModulatorUpdaterAgent, Agent> modulatorUpdaterFactory;

public:
    OpenPsiModule(CogServer&);
    virtual ~OpenPsiModule();
    virtual void init();
    virtual const char * id(void);


}; // class

} // namespace

#endif // _OPENCOG_DYNAMICS_OPENPSI_H
