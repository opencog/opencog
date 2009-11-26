/*
 * SchemeExtend.cc
 *
 * Allow C++ code to be invoked from scheme.
 *
 * Copyright (C) 2009 Linas Vepstas
 */

#include <opencog/atomspace/Handle.h>
#include <opencog/guile/SchemeSmob.h>
#include <libguile.h>

namespace opencog {

class FuncEnviron
{
	private:
		static bool is_inited;
		static void init(void);
		static SCM do_call(SCM, SCM);
		static FuncEnviron *verify_fe(SCM, const char *);

	public:
		void do_register(const char *, int);
		virtual SCM invoke (SCM) = 0;
};

template<class T>
class FuncEnv : public FuncEnviron
{
	private:
		Handle (T::*method)(Handle);
		T* that;
		const char *scheme_name;
		enum 
		{
			H_H,
		} signature;

		virtual SCM invoke (SCM args)
		{
			SCM rc = SCM_EOL;
			switch (signature)
			{
				case H_H:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					Handle rh = (that->*method)(h);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				default:
					printf ("Error! Unsupported signature: %d\n", signature);
			}
			return rc;
		}
	public:
		FuncEnv(const char *name, Handle (T::*cb)(Handle), T *data)
		{
			that = data;
			method = cb;
			scheme_name = name;
			signature = H_H;
			do_register(name, 1);
		}
};

template<class T>
inline void declare(const char *name, Handle (T::*cb)(Handle), T *data)
{
	// FuncEnv<T> *fet = new FuncEnv<T>(name, cb, data);
	new FuncEnv<T>(name, cb, data);

	// XXX fet is never freed -- we need to have it floating around forever, 
	// so that it holds the callback method. Well, I guess maybe we could
	// have the smob hang on to it, and free it with the smob_free .. ?!  XXX
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
	scm_c_define_gsubr("opencog-extension", 2,0,0, C(do_call));
}


void FuncEnviron::do_register(const char *name, int nargs)
{
	init();

	// The smob will hold a pointer to "this" -- the FuncEnviron
	SCM smob;
	SCM_NEWSMOB (smob, SchemeSmob::cog_misc_tag, this);
	SCM_SET_SMOB_FLAGS(smob, SchemeSmob::COG_EXTEND);

	// We need to give the smab a unique name. Using addr of this is 
	// sufficient for this purpose.
#define BUFLEN 40
	char buff[BUFLEN];
	snprintf(buff, BUFLEN, "cog-ext-%p", this);
	scm_c_define (buff, smob);

	std::string wrapper = "(define (";
	wrapper += name;
	for (int i=0; i<nargs; i++)
	{
		wrapper += " ";
		char arg = 'a' + i;
		wrapper += arg;
	}
	wrapper += ") (opencog-extension ";
	wrapper += buff;
	wrapper += " (list";
	for (int i=0; i<nargs; i++)
	{
		wrapper += " ";
		char arg = 'a' + i;
		wrapper += arg;
	}
	wrapper += ")))";
	scm_c_eval_string(wrapper.c_str());
printf("duuude defined %s\n", wrapper.c_str());
}

SCM FuncEnviron::do_call(SCM sfe, SCM arglist)
{
	// First, get the environ.
	FuncEnviron *fe = verify_fe(sfe, "opencog-extension");
	SCM rc = fe->invoke(arglist);
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
// Example code showing how to use the scheme-to-C++ API.

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
			if (n)
			{
				printf("Info: received the node: %s\n", n->getName().c_str());
				CogServer& cogserver = static_cast<CogServer&>(server());
				AtomSpace *as = cogserver.getAtomSpace();
				hlist = as->addLink(LIST_LINK, h);
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

	// Create some class, and assoicate one of its members with
	// a scheme function, named "bingo"
	MyTestClass *mtc = new MyTestClass();
	declare("bingo", &MyTestClass::my_func, mtc);

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
	printf("Info: returned %s\n", rslt.c_str());
	printf("Info: we are done, bye!\n");
	return  0;
}
