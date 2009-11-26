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
		printf("duuude invoked \n");
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
	FuncEnviron *fe = verify_fe(args, "opencog-extension");
	
	printf("do_call was called fe=%p\n", fe);
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

class MyTestClass
{
	private:
		int id;
	public:
		MyTestClass(int _id) { id = _id; }
		Handle my_func(Handle h)
		{
			printf("hello world %d\n", id);
			return Handle::UNDEFINED;
		}
};

int main ()
{
	SchemeEval &eval = SchemeEval::instance();

	MyTestClass *mtc = new MyTestClass(42);

	declare("bingo", &MyTestClass::my_func, mtc);

	printf("yo\n");

	std::string rslt = eval.eval("(bingo)");
	printf("duuude bingo is %d %s\n", eval.eval_error(), rslt.c_str());

	printf("bye\n");
	return  0;
}
