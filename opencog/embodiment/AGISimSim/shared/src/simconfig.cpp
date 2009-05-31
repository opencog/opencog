/***************************************************************************
 *  Configuration class.
 *
 *  Project: AgiSim
 *
 *  Fri Feb 18 11:35:16 2005
 *  Copyright  2005  Ari A. Heljakka / Novamente LLC
 *  Email [heljakka at iki dot fi]
 *																			
 *	19.01.06	FP	formatting  
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA. */


#include "simcommon.h"
#include "simconfig.h"
#include "XMLNode.h"

using std::string;

//------------------------------------------------------------------------------------------------------------
std::map<std::string, int> Config::GetInt() {
	return intconfig;
}

std::map<std::string, float> Config::GetFloat() {
	return floatconfig;
}

std::map<std::string, std::string> Config::GetString() {
	return stringconfig;
}

/**	Hidden variables:
	<AgentMax>
	<DemonMax>
*/

Config::Config()
{ }

//--- loads default - or if available from "simconfig.def" - values to ProgramVariables ---------------------
bool Config::Create()
{
	#define V(__n, __v) InsertVariable((__n), (__v))
	//Loading standard Values
	V("TicsPerFrame", "200");
	V("VolumeFadePerDistance", "10");

	V("InitialEnergy", "10000");
	V("MaxEnergy", "15000");

	V("SensePixelVision", "1");
	V("SenseObjectVision", "1");
	V("SENSEAURAL", "1");
	V("SENSESMELL", "1");
	V("SENSETASTE", "1");
	V("SENSESELF", "1");	
	V("SENSETOUCH", "0");

	V("WalkStepLength", "0.2");
	
	V("PricePerStep", "10");
	V("PricePerkgm", "1");	
	V("PricePerEat", "5");
	V("PricePerHandMove", "0.01");
	V("PricePerTurn", "0.01");	
	V("PricePerEyeMove", "0.01");	
	V("PricePerNoise", "0");
	V("PricePerMessage", "0");
	
	V("EatQuantum", "100");
	V("TurnQuantum", "0.01");
	V("HandMoveQuantum", "0.01");
	V("EyeMoveQuantum", "0.01");
	V("MoveQuantum", "0.2");
	
	V("DefaultDensity", "1000"); // kg 
	V("AgentBodyDensity", "100");
	V("AgentBodyRadius", "0.50f");
	V("MinimumEatDistance", "0.75f");	
	V("DeathDisablesAgent", "1");

  stringconfig[toupper("World")] = "hard";

//  V("World", "hard");

	V("LogLevel", "3");
	V("AgentMax", "1");
	V("DemonMax", "1");
	V("MaxMoveLength", "3.0f");
	V("UsePolygonNameInObjectVision", "0");
	V("SenseBrightnessInObjectVision", "0");
	V("ObjectVisionResolution", "20");
	V("PixelVisionResolution", "20");
	V("LogSenseData", "0");
	V("FrameUpdateDelay", "20");
	V("SocketPollingDelay","1");
	V("LogOnFile", "1");
	V("DrawGUI", "1");
	
	V("CameraRelativeX", "0.0f");
	V("CameraRelativeY", "0.6f");
	V("CameraRelativeZ", "0.0f");
	
	V("EyeThetaMin", "-0.18f");
	V("EyeThetaMax", "0.37"); //+3.14f / 8);
	V("EyePhiMin", "-3.13f");
	V("EyePhiMax", "+3.13f");
	
	V("RoomSizeX","15.0f");
	V("RoomSizeY","15.0f");
	V("RoomSizeZ","15.0f");
	
	V("MinBounceOnCollision", "0.1f");	
	V("SimWorldFileName", "simworld.def");

	if (Load())	{
		 LOG("Config", 0, "Configuration was loaded from file simconfig.def.");
		return true;
	}
	else return false;
	
/*	float pricePerStep = floatconfig["AgentBodyDensity"]
		* floatconfig["AgentBodyRadius"];
	char t[80];
	sprintf(t, "%.2f", pricePerStep);
	V("PricePerStep", t);*/
}

//------------------------------------------------------------------------------------------------------------
void Config::InsertVariable (string name, string value)
{

	const char* string_value = value.c_str();	
	int int_value = atoi(string_value);
	float float_value = (float)atof(string_value);
			
  string upperName = toupper(name);

	stringconfig[upperName] = string_value;
	intconfig[upperName] = int_value;

  floatconfig[upperName] = float_value;
			
	LOG("Config", 0, name + " loaded.");
}

//------------------------------------------------------------------------------------------------------------
bool Config::Load() {
	try	{

#ifdef ENV_VAR
		//get the env variable AGISIM, which is a path to agisim install
		char fullpath[1024];//can be [_MAX_PATH]
		char * AgisimEnv = getenv( "AGISIM" );
     	if( AgisimEnv == NULL )
		{
			LOG("ENV_VAR", 0, "No Enviournment Variable Found. Can't load config from simconfig.def"); 
			return false;
		}
		for(int i=0;i<=(int)strlen(AgisimEnv);i++){fullpath[i]=AgisimEnv[i];} //<=
#ifdef WIN32
		strcat(fullpath,"\\appdata\\defs\\simconfig.def");
#else
		strcat(fullpath,"/appdata/defs/simconfig.def");
#endif //WIN32
		FILE *f = fopen(fullpath, "rt");
#else
		FILE *f = fopen("simconfig.def", "rt");
#endif //ENV_VAR

		if (!f) 
		{
			LOG("Config", 0, "simconfig.def not found"); 
			return false;
		}

		char  buf[64001];
		fread(buf, 64000, 1, f);
		fclose(f);
	
		XMLNode     node(buf);		
		XMLVALIDATE (node, "config", throw string("Config file must have <config> root!"));
		XMLITERATE  (node, variable)
		{
			string name 	    = (*variable)->TagData().name;
			string string_value = (*variable)->TagData().textcontent;	
			
			InsertVariable (name, string_value);		
		}
	} catch (string s) { LOG("Config", 0, s); return false; }
		catch (...) { LOG("Config", 0, "Unknown exception."); return false; }
	


	return true;
}



/*	<VolumeFadePerDistance> 10 </VolumeFadePerDistance>

	<InitialEnergy> 10000 </InitialEnergy>
	<MaxEnergy> 15000 </MaxEnergy>

	<SENSETOUCH> 0 </SENSETOUCH>

	<WalkStepLength> 0.2 </WalkStepLength>

	<PricePerStep> 10 </PricePerStep>
	
	<PricePerkgm> 1 </PricePerkgm>
	<AgentBodyDensity>100 </AgentBodyDensity>
	
	<PricePerEat> 5 </PricePerEat>
	<PricePerHandMove> 0.01 </PricePerHandMove>
	<PricePerTurn> 0.01 </PricePerTurn>
	<PricePerEyeMove> 0.01 </PricePerEyeMove>
	<PricePerNoise> 0 </PricePerNoise>
	<PricePerMessage> 0 </PricePerMessage>
	
	<TurnQuantum> 0.01 </TurnQuantum>
	<HandMoveQuantum> 0.01 </HandMoveQuantum>
	<EyeMoveQuantum> 0.01 </EyeMoveQuantum>
	<MoveQuantum> 0.2 </MoveQuantum>
	
	<DefaultDensity> 100 </DefaultDensity>
	<AgentBodyRadius> 0.50f </AgentBodyRadius>
	<MinimumEatDistance> 0.75f </MinimumEatDistance>


	<UsePolygonNameInObjectVision> 0 </UsePolygonNameInObjectVision>
	
	<LogSenseData> 0 </LogSenseData>

	<EyePhiMin> -3.13f </EyePhiMin>
	<EyePhiMax> +3.13f </EyePhiMax>
	

	<SimWorldFileName> simworld.def </SimWorldFileName>
*/
