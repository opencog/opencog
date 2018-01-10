/*
 * cluster.c
 *
 * Data for related-word clusters. Meant to expand disjunct coverage
 * for the case where a parse cannot be completed without omitting
 * a word.
 *
 * Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sqlite3.h>
#include "cluster.h"

#include "build-disjuncts.h"
#include "disjunct-utils.h"
#include "structures.h"
#include "utilities.h"

struct cluster_s
{
	char * dbname;
	sqlite3 *dbconn;
	sqlite3_stmt *clu_query;
	sqlite3_stmt *dj_query;
	char *errmsg;
	int rc;
};

/* ========================================================= */

static void * db_file_open(const char * dbname, void * user_data)
{
	Cluster *c = (Cluster *) user_data;
	sqlite3 *dbconn;
	c->rc = sqlite3_open_v2(dbname, &dbconn, SQLITE_OPEN_READONLY, NULL);
	if (c->rc)
	{
		sqlite3_close(dbconn);
		return NULL;
	}

	c->dbname = strdup(dbname);
	return dbconn;
}


/**
 * Initialize the cluster statistics subsystem.
 */
Cluster * lg_cluster_new(void)
{
	int rc;

	Cluster *c = (Cluster *) malloc(sizeof(Cluster));
	c->clu_query = NULL;
	c->dj_query = NULL;
	c->errmsg = NULL;
	c->dbname = NULL;

	/* dbname = "/link-grammar/data/en/sql/clusters.db"; */
#ifdef _WIN32
#define DBNAME "sql\\clusters.db"
#else
#define DBNAME "sql/clusters.db"
#endif
	c->dbconn = object_open(DBNAME, db_file_open, c);
	if (NULL == c->dbconn)
	{
		/* Very weird .. but if the database is not found, then sqlite
		 * reports an "out of memory" error! So hide this misleading
		 * error message.
		 */
		if (SQLITE_CANTOPEN == c->rc)
		{
			prt_error("Warning: Can't open database: File not found\n"
			          "\tWas looking for: " DBNAME);
		}
		else
		{
			prt_error("Warning: Can't open database: %s\n"
			          "\tWas looking for: " DBNAME,
				sqlite3_errmsg(c->dbconn));
		}
		return c;
	}

	/* Now prepare the statements we plan to use */
	rc = sqlite3_prepare_v2(c->dbconn,
		"SELECT cluster_name FROM ClusterMembers "
		"WHERE inflected_word = ?;",
		-1, &c->clu_query, NULL);
	if (rc != SQLITE_OK)
	{
		prt_error("Error: Can't prepare the cluster member statment: %s\n",
			sqlite3_errmsg(c->dbconn));
	}

	rc = sqlite3_prepare_v2(c->dbconn,
		"SELECT disjunct, cost FROM ClusterDisjuncts "
		"WHERE cluster_name = ?;",
		-1, &c->dj_query, NULL);
	if (rc != SQLITE_OK)
	{
		prt_error("Error: Can't prepare the disjunct statment: %s\n",
			sqlite3_errmsg(c->dbconn));
	}

	prt_error("Info: Cluster grouping database found at %s\n", c->dbname);
	return c;
}

/**
 * lg_cluster_delete -- shut down the cluster statistics subsystem.
 */
void lg_cluster_delete(Cluster *c)
{
	if (NULL == c) return;

	if (c->clu_query)
	{
		sqlite3_finalize(c->clu_query);
		c->clu_query = NULL;
	}

	if (c->dj_query)
	{
		sqlite3_finalize(c->dj_query);
		c->dj_query = NULL;
	}

	if (c->dbconn)
	{
		sqlite3_close(c->dbconn);
		c->dbconn = NULL;
	}

	if (c->dbname)
	{
		free(c->dbname);
		c->dbname = NULL;
	}
	free(c);
}

/* ========================================================= */

static Exp * make_exp(const char *djstr, double cost)
{
	char * tmp;
	Exp *p1, *p2;
	E_list *l, *lhead = NULL;
	size_t len;
	const char *sp = strchr (djstr, ' ');

	Exp *e = (Exp *) malloc(sizeof(Exp));
	e->multi = 0;
	e->dir = ' ';
	e->cost = cost;

	/* If its just a single connector, then do just that */
	if (NULL == sp || 0x0 == sp[1])
	{
		e->type = CONNECTOR_type;
		if ('@' == djstr[0]) { e->multi = 1; djstr++; }
		len = strlen(djstr) - 1;
		if (sp) len--;
		e->u.string = strndup(djstr, len);
		e->dir = djstr[len];
		return e;
	}

	/* If there are multiple connectors, and them together */
	len = sp - djstr;
	tmp = strndup(djstr, len);
	p1 = make_exp(tmp, 0.0);
	free (tmp);
	p2 = make_exp(sp+1, 0.0);

	l = (E_list *) malloc(sizeof(E_list));
	l->next = lhead;
	l->e = p2;
	lhead = l;
	
	l = (E_list *) malloc(sizeof(E_list));
	l->next = lhead;
	l->e = p1;
	lhead = l;

	e->type = AND_type;
	e->u.l = lhead;
	
	return e;
}

#if NOT_NEEDED
static Exp * or_exp(Exp *p1, Exp *p2)
{
	E_list *l;
	E_list *lhead = NULL;

	if (NULL == p2) return p1;

	Exp *e = (Exp *) malloc(sizeof(Exp));
	e->multi = 0;
	e->dir = ' ';
	e->cost = 0.0;
	e->type = OR_type;

	l = (E_list *) malloc(sizeof(E_list));
	l->next = lhead;
	l->e = p2;
	lhead = l;
	
	l = (E_list *) malloc(sizeof(E_list));
	l->next = lhead;
	l->e = p1;
	lhead = l;

	e->u.l = lhead;
	return e;
}
#endif

static void free_exp(Exp *e)
{
	if (CONNECTOR_type != e->type)
	{
		E_list *l = e->u.l;
		while(l)
		{
			E_list *ln = l->next;
			free_exp(l->e);
			free(l);
			l = ln;
		}
		return;
	}

	free((char *) e->u.string);
	free(e);
}

Disjunct * lg_cluster_get_disjuncts(Cluster *c, const char * wrd)
{
	Disjunct *djl = NULL;
	int rc;
	const char * cluname;

	/* Look for a cluster containing this word */
	rc = sqlite3_bind_text(c->clu_query, 1, wrd, -1, SQLITE_STATIC);
	rc = sqlite3_step(c->clu_query);
	if (rc != SQLITE_ROW) goto noclust;

	/* Get the cluster name, and look for the disjuncts */
	cluname = sqlite3_column_text(c->clu_query,0);
	rc = sqlite3_bind_text(c->dj_query, 1, cluname, -1, SQLITE_STATIC);

	while(1)
	{
		const char *djs;
		double cost;
		Exp *e;
		Disjunct *dj;

		rc = sqlite3_step(c->dj_query);
		if (rc != SQLITE_ROW) break;
		djs = sqlite3_column_text(c->dj_query,0);
		cost = sqlite3_column_double(c->dj_query,1);

		/* All expanded disjuncts are costly! */
		// cost += 0.5;
		cost -= 6.0;
		if (cost < 0.0) cost = 0.0;

		/* Building expressions */
		e = make_exp(djs, cost);
		dj = build_disjuncts_for_exp(e, wrd, MAX_CONNECTOR_COST);
		djl = catenate_disjuncts(dj, djl);
		free_exp(e);
	}

	sqlite3_reset(c->dj_query);
	sqlite3_clear_bindings(c->dj_query);

noclust:
	sqlite3_reset(c->clu_query);
	sqlite3_clear_bindings(c->clu_query);
	return djl;
}


/* ======================= END OF FILE ===================== */
