/***************************************************************************
 *  Configuration class.        
 *
 *  Project: AgiSim
 *
 *  See implementation (.cc, .cpp) files for license details (GPL).
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *
 *	19.01.06	FP	formatting 
 ****************************************************************************/


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
