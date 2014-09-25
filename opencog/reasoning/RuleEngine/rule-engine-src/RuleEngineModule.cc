/*
 * RuleEngineModule.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 *  * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Sept 2014
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

#include "InferenceSCM.h"
#include "RuleEngineModule.h"

using namespace opencog;

DECLARE_MODULE(RuleEngineModule);

RuleEngineModule::RuleEngineModule(CogServer &cs) :
		Module(cs) {
	iscm_ = NULL;
}

RuleEngineModule::~RuleEngineModule() {
	delete iscm_;
}

void RuleEngineModule::init(void) {
	iscm_ = new InferenceSCM();
}
