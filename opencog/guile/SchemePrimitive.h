/*
 * SchemePrimitive.h
 *
 * Allow C++ code to be invoked from scheme -- 
 * by creating a new scheme primitive function.
 *
 * Copyright (C) 2009 Linas Vepstas
 */

#ifdef HAVE_GUILE

#ifndef _OPENCOG_SCHEME_PRIMITIVE_H
#define _OPENCOG_SCHEME_PRIMITIVE_H

#include <string>

#include <opencog/atomspace/Handle.h>
#include <opencog/guile/SchemeSmob.h>
#include <libguile.h>

namespace opencog {

class PrimitiveEnviron
{
	friend class SchemeEval;
	friend class SchemeSmob;
	private:
		static bool is_inited;
		static void init(void);

		static void * c_wrap_register(void *);
		void really_do_register(const char *, int);
		const char *tmp_name;
		int tmp_nargs;

		static SCM do_call(SCM, SCM);
		static PrimitiveEnviron *verify_pe(SCM, const char *);

	protected:
		void do_register(const char *, int);
		virtual SCM invoke (SCM) = 0;
		virtual const char *get_name(void) = 0;
		virtual size_t get_size(void) = 0;
		virtual ~PrimitiveEnviron();
};

template<class T>
class SchemePrimitive : public PrimitiveEnviron
{
	private:
		union
		{
			bool (T::*b_hi)(Handle, int);
			Handle (T::*h_hi)(Handle, int);
			Handle (T::*h_h)(Handle);
			Handle (T::*h_sq)(const std::string&, const HandleSeq&);
			const std::string& (T::*s_s)(const std::string&);
			void (T::*v_v)(void);
		} method;
		T* that;
		const char *scheme_name;
		enum 
		{
			B_HI,  // return boolean, take handle and int
			H_HI,  // return handle, take handle and int
			H_H,   // return handle, take handle
			H_SQ,  // return handle, take string and HandleSeq
			S_S,   // return string, take string
			V_V    // return void, take void
		} signature;

		virtual SCM invoke (SCM args)
		{
			SCM rc = SCM_EOL;
			switch (signature)
			{
				case B_HI:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					int i = scm_to_int(scm_cadr(args));
					bool b = (that->*method.b_hi)(h, i);
					if (b) { rc = SCM_BOOL_T; } else { rc = SCM_BOOL_F; }
					break;
				}
				case H_HI:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					int i = scm_to_int(scm_cadr(args));
					Handle rh = (that->*method.h_hi)(h,i);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case H_H:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					Handle rh = (that->*method.h_h)(h);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case H_SQ:
				{
					// First argument is a string
					char *lstr = scm_to_locale_string(scm_car(args));
					std::string str = lstr;
					free(lstr);

					// Second arg is a list of Handles
					SCM list = scm_cadr(args);
					if (!scm_is_pair(list))
					{
						scm_wrong_type_arg_msg(scheme_name, 2, list, "list of atom handles");
					}
					HandleSeq seq;
					while (scm_is_pair(list))
					{
						Handle h = SchemeSmob::verify_handle(scm_car(list), scheme_name);
						seq.push_back(h);
						list = SCM_CDR(list);
					}

					Handle rh = (that->*method.h_sq)(str, seq);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case S_S:
				{
					char *lstr = scm_to_locale_string(scm_car(args));
					std::string str = lstr;
					free(lstr);

					const std::string &rs = (that->*method.s_s)(str);
					rc = scm_from_locale_string(rs.c_str());
					break;
				}
				case V_V:
				{
					(that->*method.v_v)();
					break;
				}
				default:
					printf ("Error! Unsupported signature: %d\n", signature);
			}
			return rc;
		}
	protected:
		virtual const char *get_name(void) { return scheme_name; }
		virtual size_t get_size(void) { return sizeof (*this); }
	public:

#define DECLARE_CONSTR_1(SIG, LSIG, RET_TYPE, ARG_TYPE) \
	SchemePrimitive(const char *name, RET_TYPE (T::*cb)(ARG_TYPE), T *data) \
	{ \
		that = data; \
		method.LSIG = cb; \
		scheme_name = name; \
		signature = SIG; \
		do_register(name, 1); /* cb has 1 arg */ \
	}

#define DECLARE_CONSTR_2(SIG, LSIG, RET_TYPE, ARG1_TYPE, ARG2_TYPE) \
	SchemePrimitive(const char *name, RET_TYPE (T::*cb)(ARG1_TYPE, ARG2_TYPE), T *data) \
	{ \
		that = data; \
		method.LSIG = cb; \
		scheme_name = name; \
		signature = SIG; \
		do_register(name, 2); /* cb has 2 args */ \
	}

		// Declare and define the constructors for this class. They all have
		// the same basic form, except for the types.
		DECLARE_CONSTR_2(B_HI, b_hi, bool, Handle, int)
		DECLARE_CONSTR_2(H_HI, h_hi, Handle, Handle, int)
		DECLARE_CONSTR_1(H_H,  h_h,  Handle, Handle)
		DECLARE_CONSTR_1(S_S,  s_s,  const std::string&, const std::string&)
		DECLARE_CONSTR_2(H_SQ, h_sq, Handle, const std::string&, const HandleSeq&)

		SchemePrimitive(const char *name, void (T::*cb)(void), T *data)
		{
			that = data;
			method.v_v = cb;
			scheme_name = name;
			signature = V_V;
			do_register(name, 0); // cb has 0 args
		}
};

#define DECLARE_DECLARE_1(RET,ARG) \
template<class T> \
inline void define_scheme_primitive(const char *name, RET (T::*cb)(ARG), T *data) \
{ \
	/* Note: this is freed automatically by scheme garbage collection */ \
	/* when it is no longer needed. */ \
	new SchemePrimitive<T>(name, cb, data); \
}

#define DECLARE_DECLARE_2(RET,ARG1,ARG2) \
template<class T> \
inline void define_scheme_primitive(const char *name, RET (T::*cb)(ARG1,ARG2), T *data) \
{ \
	/* Note: this is freed automatically by scheme garbage collection */ \
	/* when it is no longer needed. */ \
	new SchemePrimitive<T>(name, cb, data); \
}

DECLARE_DECLARE_1(Handle, Handle)
DECLARE_DECLARE_1(const std::string&, const std::string&)
DECLARE_DECLARE_1(void, void)
DECLARE_DECLARE_2(bool, Handle, int)
DECLARE_DECLARE_2(Handle, Handle, int)
DECLARE_DECLARE_2(Handle, const std::string&, const HandleSeq&)


}

#endif // _OPENCOG_SCHEME_PRIMITIVE_H

#endif // HAVE_GUILE

