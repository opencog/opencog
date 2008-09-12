/*
 * SenseSimilaritySQL.cc
 *
 * Fetches wordnet-based sense-similarity measures from database.
 * These had been previously pre-computed. The database is generated
 * by scripts in the 'lexical attraction' package, which can be 
 * found on launchpad. The input to those scripts is, in turn, a
 * large selection of previously parsed text. 
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
		// They are also used in the literature, in e.g. Mihalcea papers.
		double hso;
		double jcn;
		double lch;
		double lesk;
		double lin;
		double path;
		double res;
		double vector;
		double wup;

		bool have_data;

		Response(void)
		{
			have_data = false;
		}

		bool row_cb(void)
		{
			rs->foreach_column(&Response::column_cb, this);
			have_data = true;
			return false;
		}

		bool column_cb(const char *colname, const char * colvalue)
		{
			if (!strcmp(colname, "hso"))
			{
				hso = atof(colvalue);
			}
			else if (!strcmp(colname, "jcn"))
			{
				jcn = atof(colvalue);
			}
			else if (!strcmp(colname, "lch"))
			{
				lch = atof(colvalue);
			}
			else if (!strcmp(colname, "lesk"))
			{
				lesk = atof(colvalue);
			}
			else if (!strcmp(colname, "lin"))
			{
				lin = atof(colvalue);
			}
			else if (!strcmp(colname, "path"))
			{
				path = atof(colvalue);
			}
			else if (!strcmp(colname, "res"))
			{
				res = atof(colvalue);
			}
			else if (!strcmp(colname, "vector"))
			{
				vector = atof(colvalue);
			}
			else if (!strcmp(colname, "wup"))
			{
				wup = atof(colvalue);
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

	escape_single_quotes(fk);
	escape_single_quotes(sk);

#define BUFSZ 500
	char qry[BUFSZ];
	snprintf(qry, BUFSZ, "SELECT * FROM SensePairScores WHERE "
		"sense_idx_a = \'%s\' AND sense_idx_b = \'%s\';",
		fk.c_str(), sk.c_str());

	Response rp;
	rp.rs = db_conn->exec(qry);
	rp.rs->foreach_row(&Response::row_cb, &rp);
	rp.rs->release();

	// If no data, return similarity of zero!
	// XXX however, what we should really do is to not that we have no
	// data, and maybe try to gather some.
	if (!rp.have_data)
	{
		SimpleTruthValue stv(0.0f, 0.9f);
		return stv;
	}

	std::string first_pos = get_part_of_speech(first_sense);
	std::string second_pos = get_part_of_speech(second_sense);

	double sim;
	if (0 == first_pos.compare(second_pos))
	{
		if (0 == first_pos.compare("noun"))
		{
			// for nouns, jcn is best, per Sinha & Mihalcea
			// Also .. use thier normalization, section 5.2
			sim = (rp.jcn - 0.04) / (0.2-0.04);
		}
		else if (0 == first_pos.compare("verb"))
		{
			// for verbs, lch is best, per Sinha & Mihalcea
			sim = (rp.lch - 0.34) / (3.33 - 0.34);
		}
	}
	else
	{
		// For all else, use lesk
		sim = rp.lesk / 240.0;
	}
	if (sim < 0.0) sim = 0.0;
	if (1.0 < sim) sim = 1.0;

#ifdef DEBUG
	if (0.0 < sim) {
		printf ("%s\n", qry);
		printf ("sim=%g\n", sim);
	}
#endif

	SimpleTruthValue stv((float) sim, 0.9f);
	return stv;
}

/* ============================== END OF FILE ====================== */
