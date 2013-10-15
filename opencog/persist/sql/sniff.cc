/*
 * FUNCTION:
 * Sniff test. Low-brow test, to see if basic atom storage is working.
 *
 * HISTORY:
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_SQL_STORAGE

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/persist/sql/AtomStorage.h>

using namespace opencog;

int atomCompare(Atom *a, Atom *b)
{
	int rc = 0;
	Link *la = dynamic_cast<Link *>(a);
	Link *lb = dynamic_cast<Link *>(b);
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
	if (la->getArity() != lb->getArity())
	{
		fprintf(stderr, "Error, arity mis-match, a=%d b=%d\n", la->getArity(), lb->getArity());
		rc --;
	}
	if (0 < la->getArity())
	{
		std::vector<Handle> outa = la->getOutgoingSet();
		std::vector<Handle> outb = lb->getOutgoingSet();
		for (int i =0; i< la->getArity(); i++)
		{
			if (outa[i] != outb[i])
			{
				fprintf(stderr, "Error, outgoing set mis-match, "
				        "i=%d a=%lx b=%lx\n", i, outa[i].value(), outb[i].value());
				rc --;
			}
		}
	}
	if (!(a->getTruthValue() == b->getTruthValue()))
	{
		TruthValuePtr ta = a->getTruthValue();
		TruthValuePtr tb = b->getTruthValue();
		fprintf(stderr, "Error, truth value miscompare, "
		        "ma=%f mb=%f ca=%f cb=%f\n",
		        ta->getMean(), tb->getMean(), ta->getCount(), tb->getCount());
		rc --;
	}
	return rc;
}


/**
 * A simple test cases that tests the save and restore of 
 * a couple of nodes and a link. Does not test atomspaces at all.
 */
#if 0
void single_atom_test(std::string id)
{
	AtomStorage *store = new AtomStorage("opencog", "linas", NULL);

	// Create an atom ... 
	Atom *a = new Node(SCHEMA_NODE, id + "someNode");
	SimpleTruthValue stv(0.55, 0.6);
	a->setTruthValue(stv);
	TLB::addAtom(a);

	// Store the atom ... 
	store->storeAtom(a);

	// Fetch it back ...
	Handle h = a->getHandle();
	Atom *b = store->getAtom(h);

	// Are they equal ??
	int rc = atomCompare(a,b);
	if (!rc) 
	{
		printf("atom compare success\n");
	}

	// Create a second atom, connect it to the first
	// with a link. Save it, fetch it ... are they equal?
	Atom *a2 = new Node(SCHEMA_NODE, id + "otherNode");
	TLB::addAtom(a2);
	store->storeAtom(a2);

	std::vector<Handle> hvec;
	hvec.push_back(a->getHandle());
	hvec.push_back(a2->getHandle());

	Link *l = new Link(SET_LINK, hvec);
	TLB::addAtom(l);
	store->storeAtom(l);

	Atom *lb = store->getAtom(l->getHandle());
	rc = atomCompare(l,lb);
	if (!rc) 
	{
		printf("link compare success\n");
	}

	delete store;
}

void add_to_table(AtomTable *table, std::string id)
{
	// Create an atom ... 
	Atom *a = new Node(SCHEMA_NODE, id + "fromNode");
	SimpleTruthValue stv(0.11, 33);
	a->setTruthValue(stv);
	table->add(a);

	Atom *a2 = new Node(SCHEMA_NODE, id + "toNode");
	SimpleTruthValue stv2(0.22, 66);
	a2->setTruthValue(stv2);
	table->add(a2);

	Atom *a3 = new Node(SCHEMA_NODE, id + "third wheel");
	SimpleTruthValue stv3(0.33, 99);
	a3->setTruthValue(stv3);
	table->add(a3);

	std::vector<Handle> hvec;
	hvec.push_back(a->getHandle());
	hvec.push_back(a2->getHandle());
	hvec.push_back(a3->getHandle());

	Link *l = new Link(SET_LINK, hvec);
	table->add(l);
}
#endif


int main ()
{
#if 0
	single_atom_test("aaa ");
	single_atom_test("bbb ");
	single_atom_test("ccc ");
	single_atom_test("ddd ");
	single_atom_test("eee ");
#endif

#if 1
	AtomStorage *store = new AtomStorage("opencog", "linas", NULL);

	AtomTable *table = new AtomTable();
	store->load(*table);

	printf("Printing table:\n");
	table->print();

	delete store;
#endif

#if 0
	AtomStorage *store = new AtomStorage("opencog", "linas", NULL);

	AtomTable *table = new AtomTable();
	add_to_table(table, "aaa ");
	add_to_table(table, "bbb ");
	add_to_table(table, "ccc ");
	add_to_table(table, "ddd ");
	add_to_table(table, "eee ");

	store->store(*table);

	delete store;
#endif
	return 0;
}

#else /* HAVE_SQL_STORAGE */
int main () { return 1; }
#endif /* HAVE_SQL_STORAGE */
/* ============================= END OF FILE ================= */
