/*
 * SchemeExtend.cc
 *
 * Allow C++ code to be invoked from scheme.
 *
 * Copyright (C) 2009 Linas Vepstas
 */

template<class T>
inline void declare(const char *name, Handle (T::*cb)(Handle), T *data)
{
}


class SchemeExtend
{
	private:
	public:
		typedef Handle (*H_V)(const HandleSeq &);
		void declare (const char *, H_V); 
};


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
