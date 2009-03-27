/*
 * opencog/embodiment/AGISimSim/shared/include/simconfig.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Ari A. Heljakka
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


#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <string>
#include "singleton.h"
#include "utils.h"
#include "log.h"
#include <string.h>


//------------------------------------------------------------------------------------------------------------
// Only compilation options should be given here.
// Everything else is set dynamically in simconfig.cc!
//------------------------------------------------------------------------------------------------------------
#define LOCAL_SERVER 1

//------------------------------------------------------------------------------------------------------------
/** @class Config
	\brief The storage for configuration variables.
	
	The configuration is semi-dynamic.
	You can access a variable by giving its name and type, in the following way:

	Int values: by \b IntConfig("name")

	Float values: by \b FloatConfig("name")

	String values: by \b StringConfig("name")

	Using normal strings for variables avoids awkward re-compilation.
	However, if you accidentally use a variable name that doesn't exist,
	an error is logged and an exception is throw. Therefore, you should
	never make variable names <i>dynamic</i>. (Nor should there ever be a
	reason for such.) */
//------------------------------------------------------------------------------------------------------------		
class Config : public Singleton<Config> {
private:	
  std::map<std::string,int> intconfig;
  std::map<std::string,float> floatconfig;
  std::map<std::string,std::string> stringconfig;

	Config();
	bool Load();

	/** Set a variable to a given value.
		Should only be used from class CommandConfig */
	void InsertVariable(string name, string value);
public:
	/** Initialize the defaults and load in the config file. */
	bool Create();

  std::map<std::string, int> 		  GetInt();		//Stores IntValues form simconfig.def file
	std::map<std::string, float> 	  GetFloat();
	std::map<std::string, std::string> GetString();

	friend class Singleton<Config>;
	friend class CommandConfig;
};

#define IntConfig(param)    (Config::Instance().GetInt()[toupper(param)])
#define FloatConfig(param)  (Config::Instance().GetFloat()[toupper(param)])
#define StringConfig(param) (Config::Instance().GetString()[toupper(param)])

#endif
