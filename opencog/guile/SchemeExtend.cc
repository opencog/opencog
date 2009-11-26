/*
 * SchemeExtend.cc
 *
 * Allow C++ code to be invoked from scheme.
 *
 * Copyright (C) 2009 Linas Vepstas
 */

#include <opencog/atomspace/Handle.h>
#include <libguile.h>

namespace opencog {

class FuncEnviron
{
	private:
		static bool is_inited;
		static void init(void);
		static SCM do_call(SCM);
		static FuncEnviron *verify_fe(SCM, const char *);

	public:
		void do_register(const char *);
		virtual SCM invoke (SCM) = 0;
};

template<class T>
class FuncEnv : public FuncEnviron
{
	virtual SCM invoke (SCM args)
	{
		(that->*method)(Handle::UNDEFINED);
		return SCM_EOL;
	}
	public:
		Handle (T::*method)(Handle);
		T* that;
};

template<class T>
inline void declare(const char *name, Handle (T::*cb)(Handle), T *data)
{
	FuncEnv<T> *fet = new FuncEnv<T>;
	fet->do_register(name);
	fet->method = cb;
	fet->that = data;
}

};


// ======================================================================

#include "SchemeEval.h"
#include "SchemeSmob.h"

using namespace opencog;

bool FuncEnviron::is_inited = false;

#define C(X) ((SCM (*) ()) X)

void FuncEnviron::init(void)
{
	if (is_inited) return;
	is_inited = true;
	scm_c_define_gsubr("opencog-extension", 1,1,0, C(do_call));
}


void FuncEnviron::do_register(const char * name)
{
	init();
	SCM smob;
	SCM_NEWSMOB (smob, SchemeSmob::cog_misc_tag, this);
	SCM_SET_SMOB_FLAGS(smob, SchemeSmob::COG_EXTEND);

#define BUFLEN 512
	char buff[BUFLEN];
	snprintf(buff, BUFLEN, "cog-ext-%p", this);
	scm_c_define (buff, smob);

	snprintf(buff, BUFLEN, "(define (%s) (opencog-extension cog-ext-%p))", name, this);
	scm_c_eval_string(buff);
}

SCM FuncEnviron::do_call(SCM args)
{
	// XXX do general args ... 
	FuncEnviron *fe = verify_fe(args, "opencog-extension");
	SCM rc = fe->invoke(SCM_EOL);
	return rc;
}

FuncEnviron * FuncEnviron::verify_fe(SCM sfe, const char *subrname)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, sfe))
		scm_wrong_type_arg_msg(subrname, 2, sfe, "opencog primitive function");

	scm_t_bits misctype = SCM_SMOB_FLAGS(sfe);
	if (SchemeSmob::COG_EXTEND != misctype)
		scm_wrong_type_arg_msg(subrname, 2, sfe, "opencog primitive function");

	FuncEnviron * fe = (FuncEnviron *) SCM_SMOB_DATA(sfe);
	return fe;
}

// ===============================================================
// Example code

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>
#include <opencog/server/CogServer.h>


class MyTestClass
{
	public:
		Handle my_func(Handle h)
		{
			Handle hlist = Handle::UNDEFINED;
			Atom *a = TLB::getAtom(h);
			Node *n = dynamic_cast<Node *>(a);
			printf("hello world %p %p\n", a, n);
			if (n)
			{
				printf("Info: received the node: %s\n", n->getName().c_str());
				CogServer& cogserver = static_cast<CogServer&>(server());
				AtomSpace *as = cogserver.getAtomSpace();
				Handle hlist = as->addLink(LIST_LINK, h);
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

	SchemeEval &eval = SchemeEval::instance();

	MyTestClass *mtc = new MyTestClass();

	declare("bingo", &MyTestClass::my_func, mtc);

	printf("yo\n");

	eval.eval("(define nnn (cog-new-node 'ConceptNode \"Hello World!\"))");
	std::string rslt = eval.eval("(bingo nnn)");
	if (eval.eval_error())
	{
		printf("Error: failed evaluation: %s\n", rslt.c_str());
	}

	printf("bye\n");
	return  0;
}
