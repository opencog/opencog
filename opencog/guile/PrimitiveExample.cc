/*
 * PrimitiveExample.cc
 *
 * Example code showing how declare a C++ method so that it can 
 * be called from scheme.
 *
 * Copyright (C) 2009 Linas Vepstas
 */


#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/guile/SchemePrimitive.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

// Some example class
class MyTestClass
{
	private:
		int id;  // some value in the instance
	public:

		MyTestClass(int _id) { id = _id; }

		// An example method -- accepts a handle, and wraps it 
		// with a ListLink.
		Handle my_func(Handle h)
		{
			Handle hlist = Handle::UNDEFINED;
			Atom *a = TLB::getAtom(h);
			Node *n = dynamic_cast<Node *>(a);
			if (n)
			{
				printf("Info: my_func instance %d received the node: %s\n", id, n->getName().c_str());
				CogServer& cogserver = static_cast<CogServer&>(server());
				AtomSpace *as = cogserver.getAtomSpace();
				hlist = as->addLink(LIST_LINK, h);
			}
			else
			{
				printf("Warning: my_func instance %d called with invalid handle\n", id);
			}
			return hlist;
		}
};

int main ()
{
	// Need to access the atomspace to get it to initialize itself.
	CogServer& cogserver = static_cast<CogServer&>(server());
	// AtomSpace *as = cogserver.getAtomSpace();
	cogserver.getAtomSpace();

	// Do this early, so that the scheme system is initialized.
	SchemeEval &eval = SchemeEval::instance();

	printf("\nInfo: Start creating a scheme call into C++\n");

	// Create the example class, and define a scheme function,
	// named "bingo", that will call one of its methods
	MyTestClass *mtc = new MyTestClass(42);
	define_scheme_primitive("bingo", &MyTestClass::my_func, mtc);

	// Now, call bingo, with a reasonable argument. Since 
	// MyTestClass::my_func is expecting a handle, we better pass
	// bingo a handle.
	eval.eval("(define nnn (cog-new-node 'ConceptNode \"Hello World!\"))");
	std::string rslt = eval.eval("(bingo nnn)");
	if (eval.eval_error())
	{
		printf("Error: failed evaluation\n");
	}

	// Print the result of calling MyTestClass::my_func
	printf("Info: Result of scheme evaluation is %s", rslt.c_str());
	printf("Info: We are done, bye!\n");
	return  0;
}

/*
todo
-- update README
-- add nil's signature
-- kill AdHoc.cc
-- publish new README on wiki.
-- catch OPENCOG_ASSERT per nill, and print at shell prompt.

*/
