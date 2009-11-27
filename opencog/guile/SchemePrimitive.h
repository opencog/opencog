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
	private:
		static bool is_inited;
		static void init(void);

		static SCM do_call(SCM, SCM);
		static PrimitiveEnviron *verify_pe(SCM, const char *);

	protected:
		void do_register(const char *, int);
		virtual SCM invoke (SCM) = 0;
	public:
		virtual ~PrimitiveEnviron();
		virtual const char *get_name(void) = 0;
};

template<class T>
class SchemePrimitive : public PrimitiveEnviron
{
	private:
		union
		{
			Handle (T::*h_h)(Handle);
			bool (T::*b_hi)(Handle, int);
		} method;
		T* that;
		const char *scheme_name;
		enum 
		{
			H_H,  // return handle, take handle
			B_HI, // return boolean, take handle and int
		} signature;

		virtual SCM invoke (SCM args)
		{
			SCM rc = SCM_EOL;
			switch (signature)
			{
				case H_H:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					Handle rh = (that->*method.h_h)(h);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case B_HI:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					int i = scm_to_int(scm_cadr(args));
					bool b = (that->*method.b_hi)(h, i);
					if (b) { rc = SCM_BOOL_T; } else { rc = SCM_BOOL_F; }
					break;
				}
				default:
					printf ("Error! Unsupported signature: %d\n", signature);
			}
			return rc;
		}
	public:
		SchemePrimitive(const char *name, Handle (T::*cb)(Handle), T *data)
		{
			that = data;
			method.h_h = cb;
			scheme_name = name;
			signature = H_H;
			do_register(name, 1); // cb has 1 arg
		}
		SchemePrimitive(const char *name, bool (T::*cb)(Handle, int), T *data)
		{
			that = data;
			method.b_hi = cb;
			scheme_name = name;
			signature = B_HI;
			do_register(name, 2); // cb has 2 arg
		}
		virtual const char *get_name(void) { return scheme_name; }
};

template<class T>
inline void declare(const char *name, Handle (T::*cb)(Handle), T *data)
{
	// Note: this is freed automatically by scheme garbage collection
	// when it is no longer needed. 
	new SchemePrimitive<T>(name, cb, data);
}
template<class T>
inline void declare(const char *name, bool (T::*cb)(Handle, int), T *data)
{
	new SchemePrimitive<T>(name, cb, data);
}


};

#endif // _OPENCOG_SCHEME_PRIMITIVE_H

#endif // HAVE_GUILE

