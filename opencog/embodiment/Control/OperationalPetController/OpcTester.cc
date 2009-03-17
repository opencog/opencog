#include "OPC.h"
#include <SystemParameters.h>
#include <StringMessage.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <LADSUtil/Logger.h>

using namespace std;

using namespace OperationalPetController;

int main()
{
	Control::SystemParameters sp;
	OPC * opc;	
    MessagingSystem::StringMessage *msg = new MessagingSystem::StringMessage("", "", "");
  	int length;
  	char * buffer;	
	
  	ifstream is;
  	is.open ("arquivo.xml", ios::binary );

  	// get length of file:
  	is.seekg (0, ios::end);
  	length = is.tellg();
  	is.seekg (0, ios::beg);
  

  	// allocate memory:
  	buffer = new char [length];

  	// read data as a block:
  	is.read (buffer,length);
  	is.close();
  	string xml = buffer;
  		
	//char * file = "arquivo.xml";
	//string xml = "<?xml version=\"1.0\"?><instruction pet-id=\"1\" avatar-id=\"2\" timestamp=\"2007-06-20\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:SchemaLocation=\"http://www.electricsheepcompany.com/xml/ns/PetProxy /home/dlopes/projetos/petaverse/trunk/Petaverse/build/src/PetController/petBrain.xsd\">start learning!</instruction><instruction pet-id=\"1\" avatar-id=\"2\" timestamp=\"2007-06-20\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:SchemaLocation=\"http://www.electricsheepcompany.com/xml/ns/PetProxy /home/dlopes/projetos/petaverse/trunk/Petaverse/build/src/PetController/petBrain.xsd\">stop learning</instruction>";	
	
	opc = new OPC("teste-opc", "127.0.0.1", 4000, "1", "2", "pet", "neutral", sp);
	msg->setMessage(xml);
	opc->processNextMessage(msg);
	
	delete[] buffer;
    delete opc;
	return 0;  
}

