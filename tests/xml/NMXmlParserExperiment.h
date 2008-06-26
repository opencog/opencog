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

class NMXmlParserExperiment {

private:

	static int currentExperiment;
	static const char *expContents[NNMXMLXMLEXPERIMENTS];

	//CHECKING
	static bool checkExp0();
	static bool checkExp1();

	static opencog::Handle sport;
	static opencog::Handle soccer;
	static opencog::Handle link_sport_socker;
	static opencog::Handle hihger_order_link;

    static opencog::AtomSpace* atomSpace;

public:

    static bool noCheck;
	static char *currentFileName;

	static void initStaticVars();
	static int getNExperiments();
	static void createExperiment(int, opencog::AtomSpace*);
	static bool checkExperiment();
	static opencog::AtomSpace* destroyExperiment(bool = true);
	static opencog::AtomSpace* cleanupAtomSpace();
    static opencog::AtomSpace* getAtomSpace();
};

#endif //NMXMLPARSEREXPERIMENT_H
