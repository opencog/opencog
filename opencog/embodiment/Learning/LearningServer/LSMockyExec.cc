/*
 * opencog/embodiment/Learning/LearningServer/LSMockyExec.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#include <SystemParameters.h>
#include <cstdlib>
#include "util/files.h"
#include "LSMocky.h"

using namespace LearningServer;

int main(int argc, char *argv[]) {
  
	Control::SystemParameters parameters;

    // if exists load file with configuration parameters 
    // IMPORTANT: this file should be the same for all executables that create
    // a systemParameter object. 
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    server(LSMocky::createInstance);
    LSMocky& ls = static_cast<LSMocky&>(server());
  	ls.init(parameters.get("LS_ID"), 
  			parameters.get("LS_IP"), 
  			std::atoi(parameters.get("LS_PORT").c_str()), 
  			parameters);
  	ls.serverLoop();
  	return 0;
}
