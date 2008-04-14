/*
 * FUNCTION:
 * sniff test.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "AtomStorage.h"
#include "Atom.h"
#include "Node.h"
#include "TLB.h"

int atomCompare(Atom *a, Atom *b)
{
	int rc = 0;
	if (NULL == b)
	{
		fprintf(stderr, "Error: No atom found\n");
		return -1;
	}

	if (a->getType() != b->getType())
	{
		fprintf(stderr, "Error, type mis-match, a=%d b=%d\n", a->getType(), b->getType());
		rc --;
	}
	return rc;
}


int main ()
{
	AtomStorage *store = new AtomStorage();

	Atom *a = new Node(SCHEMA_NODE, "someNode");
printf ("hello\n");

	store->storeAtom(a);

	Handle h = TLB::getHandle(a);
	Atom *b = store->getAtom(h);

	int rc = atomCompare(a,b);
	if (!rc) 
	{
		printf("atom compare success\n");
	}

	return 0;
}

/* ============================= END OF FILE ================= */
