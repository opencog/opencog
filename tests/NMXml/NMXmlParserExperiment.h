/**
 * NMXmlParserExperiment.h
 *
 *
 * Copyright(c) 2003 Rodrigo Barra
 * All rights reserved.
 */

#ifndef NMXMLPARSEREXPERIMENT_H
#define NMXMLPARSEREXPERIMENT_H
#include <types.h>
#include <AtomSpace.h>
#include <HandleEntry.h>

#define NNMXMLXMLEXPERIMENTS 1

class NMXmlParserExperiment{
private:
	static int currentExperiment;
    
	static char *expContents[NNMXMLXMLEXPERIMENTS];

	//CHECKING
	static bool checkExp0();
	static bool checkExp1();

	static Handle sport;
	static Handle soccer;
	static Handle link_sport_socker;
	static Handle hihger_order_link;

    static AtomSpace* atomSpace;

public:
    static bool noCheck;
    
	static char *currentFileName;

	static void initStaticVars();
	static int getNExperiments();
	static void createExperiment(int, AtomSpace*);
	static bool checkExperiment();
	static AtomSpace* destroyExperiment(bool = true);
	static AtomSpace* cleanupAtomSpace();
    static AtomSpace* getAtomSpace();
};

#endif //NMXMLPARSEREXPERIMENT_H
