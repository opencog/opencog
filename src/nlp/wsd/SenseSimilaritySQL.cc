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

class SenseSimilaritySQL::Response
{
	public:
		ODBCRecordSet *rs;

		// These names exactly mirror those of the WordNet::Similarity
		// package (and were computed by that package, and stuffed into DB)
		double hso;
		double jcn;
		double lch;
		double lesk;
		double lin;
		double path;
		double res;
		double vector;
		double wup;

		bool row_cb(void)
		{
			rs->foreach_column(&Response::column_cb, this);
			return false;
		}

		bool column_cb(const char *colname, const char * colvalue)
		{
			if (!strcmp(colname, "lch"))
			{
				lch = atof(colvalue);
printf ("duuude lch=%g\n", lch);
			}
			return false;
		}
};

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

	Node *fn = dynamic_cast<Node *>(TLB::getAtom(first_sense));
	Node *sn = dynamic_cast<Node *>(TLB::getAtom(second_sense));
	std::string fk = fn->getName();
	std::string sk = sn->getName();

#define BUFSZ 500
	char qry[BUFSZ];
	snprintf(qry, BUFSZ, "SELECT * FROM SensePairScores WHERE "
		"sense_idx_a = \'%s\' AND sense_idx_b = \'%s\';",
		fk.c_str(), sk.c_str());

	printf ("Yeahhh %s\n", qry);

	Response rp;
	rp.rs = db_conn->exec(qry);
	rp.rs->foreach_row(&Response::row_cb, &rp);
	rp.rs->release();

	std::string first_pos = get_part_of_speech(first_sense);
	std::string second_pos = get_part_of_speech(second_sense);

	if (0 == first_pos.compare(second_pos))
	{
		if (0 == first_pos.compare("noun"))
		{
		}
		else if (0 == first_pos.compare("verb"))
		{
		}
	}

	double sim = 0.5;
	SimpleTruthValue stv((float) sim, 0.9f);
	return stv;
}

/* ============================== END OF FILE ====================== */
