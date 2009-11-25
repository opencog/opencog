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
	public:
		void do_register(const char *);
		virtual SCM invoke (SCM) = 0;
};

template<class T>
class FuncEnv : public FuncEnviron
{
	virtual SCM invoke (SCM args)
	{
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

void FuncEnviron::do_register(const char * name)
{
	SCM smob;
	SCM_NEWSMOB (smob, SchemeSmob::cog_misc_tag, this);
	SCM_SET_SMOB_FLAGS(smob, SchemeSmob::COG_EXTEND);
}


#if 0
struct FuncEnv
{
	funcptr
	const char *name;
	void * that;	
};

FuncEnv * verify_fe(SCM sfe, const char *subrname)
{
	if (!SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, sfe))
		scm_wrong_type_arg_msg(subrname, 2, sfe, "opencog primitive function");

	scm_t_bits misctype = SCM_SMOB_FLAGS(sfe);
	if (COG_EXTEND != misctype)
		scm_wrong_type_arg_msg(subrname, 2, sfe, "opencog primitive function");

	FuncEnv * fe = (FuncEnv *) SCM_SMOB_DATA(sfe);
	return fe;
}

SCM do_call (SCM env, SCM args)
{
	FuncEnv *fe = verify_fe(env, xxxx);

	fe->funcptr(args);

	return SCM_XXX;
}

void init (void)
{
	scm_c_define_gsubr("opencog-extension", 1,1,0, C(do_call));
}

void SchemeExtend::declare(const char * name, H_V func, void *user_data)
{
	FuncEnv *fe = new FuncEnv;
	fe->name = name;
	fe->funcptr = func;
	fe->that;

	SCM smob;
	SCM_NEWSMOB (smob, cog_misc_tag, fe);
	SCM_SET_SMOB_FLAGS(smob, COG_EXTEND);
	
}
#endif 

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

	printf("yo\n");
	MyTestClass *mtc = new MyTestClass(42);

	printf("yo\n");
	declare("bingo", &MyTestClass::my_func, mtc);

	printf("yo\n");

	return  0;
}
