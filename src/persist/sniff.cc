/*
 * FUNCTION:
 * Sniff test. Low-brow test, to see if basic atom storage is working.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "AtomStorage.h"
#include "Atom.h"
#include "Link.h"
#include "Node.h"
#include "SimpleTruthValue.h"
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
	if (a->getArity() != b->getArity())
	{
		fprintf(stderr, "Error, arity mis-match, a=%d b=%d\n", a->getArity(), b->getArity());
		rc --;
	}
	if (0 < a->getArity())
	{
		std::vector<Handle> outa = a->getOutgoingSet();
		std::vector<Handle> outb = b->getOutgoingSet();
		for (int i =0; i< a->getArity(); i++)
		{
			if (outa[i] != outb[i])
			{
				fprintf(stderr, "Error, outgoing set mis-match, "
				        "i=%d a=%lx b=%lx\n", i, outa[i], outb[i]);
				rc --;
			}
		}
	}
	if (!(a->getTruthValue() == b->getTruthValue()))
	{
		const TruthValue &ta = a->getTruthValue();
		const TruthValue &tb = b->getTruthValue();
		fprintf(stderr, "Error, truth value miscompare, "
		        "ma=%f mb=%f ca=%f cb=%f\n",
		        ta.getMean(), tb.getMean(), ta.getCount(), tb.getCount());
		rc --;
	}
	return rc;
}


/**
 * A simple test cases that tests the save and restore of 
 * a couple of nodes and a link. Does not test atomspaces at all.
 */
void single_atom_test(void)
{
	AtomStorage *store = new AtomStorage("opencog", "linas", NULL);

	// Create an atom ... 
	Atom *a = new Node(SCHEMA_NODE, "someNode");
	SimpleTruthValue stv(0.55, 0.6);
	a->setTruthValue(stv);
	TLB::addAtom(a);

	// Store the atom ... 
	store->storeAtom(a);

	// Fetch it back ...
	Handle h = TLB::getHandle(a);
	Atom *b = store->getAtom(h);

	// Are they equal ??
	int rc = atomCompare(a,b);
	if (!rc) 
	{
		printf("atom compare success\n");
	}

	// Create a second atom, connect it to the first
	// with a link. Save it, fetch it ... are they equal?
	Atom *a2 = new Node(SCHEMA_NODE, "otherNode");
	TLB::addAtom(a2);
	store->storeAtom(a2);

	std::vector<Handle> hvec;
	hvec.push_back(TLB::getHandle(a));
	hvec.push_back(TLB::getHandle(a2));

	Link *l = new Link(SET_LINK, hvec);
	TLB::addAtom(l);
	store->storeAtom(l);

	Atom *lb = store->getAtom(TLB::getHandle(l));
	rc = atomCompare(l,lb);
	if (!rc) 
	{
		printf("link compare success\n");
	}

	delete store;
}


int main ()
{
	single_atom_test();

	AtomStorage *store = new AtomStorage("opencog", "linas", NULL);

	AtomTable *table = new AtomTable();
	store->load(table);

	table->print();

	delete store;
	return 0;
}

/* ============================= END OF FILE ================= */
