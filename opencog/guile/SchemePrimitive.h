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
			Handle (T::*h_h)(Handle);
			void (T::*v_v)(void);
		} method;
		T* that;
		const char *scheme_name;
		enum 
		{
			B_HI,  // return boolean, take handle and int
			H_H,  // return handle, take handle
			V_V  // return void, take void
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
				case H_H:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					Handle rh = (that->*method.h_h)(h);
					rc = SchemeSmob::handle_to_scm(rh);
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
		SchemePrimitive(const char *name, bool (T::*cb)(Handle, int), T *data)
		{
			that = data;
			method.b_hi = cb;
			scheme_name = name;
			signature = B_HI;
			do_register(name, 2); // cb has 2 args
		}
		SchemePrimitive(const char *name, Handle (T::*cb)(Handle), T *data)
		{
			that = data;
			method.h_h = cb;
			scheme_name = name;
			signature = H_H;
			do_register(name, 1); // cb has 1 arg
		}
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
inline void declare(const char *name, RET (T::*cb)(ARG), T *data) \
{ \
	/* Note: this is freed automatically by scheme garbage collection */ \
	/* when it is no longer needed. */ \
	new SchemePrimitive<T>(name, cb, data); \
}

DECLARE_DECLARE_1(Handle, Handle)
DECLARE_DECLARE_1(void, void)

template<class T>
inline void declare(const char *name, bool (T::*cb)(Handle, int), T *data)
{
	// Note: this is freed automatically by scheme garbage collection
	// when it is no longer needed. 
	new SchemePrimitive<T>(name, cb, data);
}


}

#endif // _OPENCOG_SCHEME_PRIMITIVE_H

#endif // HAVE_GUILE

