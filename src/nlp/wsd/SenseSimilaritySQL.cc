/*
 * SenseSimilaritySQL.cc
 *
 * Fetches wordnet-based sense-similarity measures from database.
 * These had been previous pre-computed.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include "platform.h"
#include <stdio.h>
#include <math.h>

#include "SenseSimilaritySQL.h"
#include "ForeachWord.h"
#include "Node.h"
#include "odbcxx.h"
#include "SimpleTruthValue.h"

using namespace opencog;

#define DEBUG

SenseSimilaritySQL::SenseSimilaritySQL(void)
{
	// XXX fix me no hard coded information, please ...
	const char * dbname = "lexat";
	const char * username = "linas";
	const char * authentication = "asdf";

   db_conn = new ODBCConnection(dbname, username, authentication);
}

SenseSimilaritySQL::~SenseSimilaritySQL()
{
	// Deleting will close the open connection
	delete db_conn;
}

SimpleTruthValue SenseSimilaritySQL::similarity(Handle fs, Handle ss)
{
	Handle first_sense = fs;
	Handle second_sense = ss;

	std::string first_pos = get_part_of_speech(first_sense);
	std::string second_pos = get_part_of_speech(second_sense);

#if 0
	if ((0 != first_pos.compare(second_pos)) ||
	    (0 == first_pos.compare("adj")) ||
	    (0 == first_pos.compare("adv")))
	{
		SimpleTruthValue stv(0.0, 0.5);
		return stv;
	}
#endif

	printf ("Ola !! %s %s \n", first_pos.c_str(), second_pos.c_str());

	Node *fn = dynamic_cast<Node *>(TLB::getAtom(first_sense));
	Node *sn = dynamic_cast<Node *>(TLB::getAtom(second_sense));
	std::string fk = fn->getName();
	std::string sk = sn->getName();

	printf ("Yeahhh %s %s\n", fk.c_str(), sk.c_str());

	double sim = 0.5;
	SimpleTruthValue stv((float) sim, 0.9f);
	return stv;
}

/* ============================== END OF FILE ====================== */
