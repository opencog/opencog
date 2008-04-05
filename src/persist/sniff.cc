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

int main ()
{
	AtomStorage *store = new AtomStorage();

	Atom *a = new Node(SCHEMA_NODE, "someNode");
printf ("hello\n");

	store->storeAtom(a);

	Handle h = TLB::getHandle(a);
	store->getAtom(h);


	return 0;
}

/* ============================= END OF FILE ================= */
