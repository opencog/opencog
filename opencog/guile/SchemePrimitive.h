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
#include <opencog/atomspace/ClassServer.h>

namespace opencog {
/** \addtogroup grp_smob
 *  @{
 */

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

//! SchemePrimitive -- template class used for defining a new primitve
//!
//! This template may be used to wrap a C++ object in such a way that it
//! can be invoked from scheme code.  The file "PrimitiveExample.cc"
//! provides a detailed example of how to do this, and how to invoke the
//! resulting primitive.
//!
//! This template has a handful of pre-defined signatures. If you cannot
//! find the signature that you need, you will need to extend this
//! template a bit to add the needed signature. This shouldn't be hard;
//! just work from the existing examples; please keep things in
//! alphabetical order.
//
template<class T>
class SchemePrimitive : public PrimitiveEnviron
{
	private:
		union
		{
			// signature naming convention:
			// b == bool
			// d == double
			// h == handle
			// i == int
			// q == HandleSeq
			// s == string
			// t == Type
			// v == void
			// Extend the above, if required.

			// Below is the list of currently supported signatures.
			// Extend as needed.
			bool (T::*b_hi)(Handle, int);
			double (T::*d_hht)(Handle, Handle, Type);
			double (T::*d_hhtb)(Handle, Handle, Type, bool);
			Handle (T::*h_h)(Handle);
			Handle (T::*h_hi)(Handle, int);
			Handle (T::*h_sq)(const std::string&, const HandleSeq&);
			Handle (T::*h_sqq)(const std::string&, const HandleSeq&, const HandleSeq&);
			HandleSeq (T::*q_hti)(Handle, Type, int);
			HandleSeq (T::*q_htib)(Handle, Type, int, bool);
			const std::string& (T::*s_s)(const std::string&);
			void (T::*v_h)(Handle);
			void (T::*v_t)(Type);
			void (T::*v_ti)(Type, int);
			void (T::*v_tidi)(Type, int, double, int);
			void (T::*v_v)(void);
		} method;
		T* that;
		const char *scheme_name;
		enum 
		{
			B_HI,  // return boolean, take handle and int
			D_HHT, // return double, take handle, handle, and type
			D_HHTB,// return double, take handle, handle, and type
			H_H,   // return handle, take handle
			H_HI,  // return handle, take handle and int
			H_SQ,  // return handle, take string and HandleSeq
			H_SQQ, // return handle, take string, HandleSeq and HandleSeq
			Q_HTI, // return HandleSeq, take handle, type, and int
			Q_HTIB,// return HandleSeq, take handle, type, and bool
			S_S,   // return string, take string
			V_H,   // return void, take Handle
			V_T,   // return void, take Type
			V_TI,  // return void, take Type and int
			V_TIDI,// return void, take Type, int, double, and int
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
					int i = SchemeSmob::verify_int(scm_cadr(args), scheme_name, 2);
					bool b = (that->*method.b_hi)(h, i);
					if (b) { rc = SCM_BOOL_T; } else { rc = SCM_BOOL_F; }
					break;
				}
				case D_HHT:
				{
					Handle h1 = SchemeSmob::verify_handle(scm_car(args), scheme_name, 1);
					Handle h2 = SchemeSmob::verify_handle(scm_cadr(args), scheme_name, 2);
					Type t = SchemeSmob::verify_atom_type(scm_caddr(args), scheme_name, 3);
					
					double d = (that->*method.d_hht)(h1,h2,t);
					rc = scm_from_double(d);
					break;
				}
				case D_HHTB:
				{
					Handle h1 = SchemeSmob::verify_handle(scm_car(args), scheme_name, 1);
					Handle h2 = SchemeSmob::verify_handle(scm_cadr(args), scheme_name, 2);
					Type t = SchemeSmob::verify_atom_type(scm_caddr(args), scheme_name, 3);
					bool b = scm_to_bool(scm_cadddr(args));
					
					double d = (that->*method.d_hhtb)(h1,h2,t,b);
					rc = scm_from_double(d);
					break;
				}
				case H_H:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					Handle rh = (that->*method.h_h)(h);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case H_HI:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					int i = SchemeSmob::verify_int(scm_cadr(args), scheme_name, 2);
					Handle rh = (that->*method.h_hi)(h,i);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case H_SQ:
				{
					// First argument is a string
					std::string str = SchemeSmob::verify_string(scm_car(args), scheme_name, 1);

					// Second arg is a list of Handles
					SCM list = scm_cadr(args);
					HandleSeq seq = SchemeSmob::verify_handle_list(list, scheme_name, 2);

					Handle rh = (that->*method.h_sq)(str, seq);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case H_SQQ:
				{
					// First argument is a string
					std::string str = SchemeSmob::verify_string(scm_car(args), scheme_name, 1);

					// Second arg is a list of Handles
					SCM list1 = scm_cadr(args);
					HandleSeq seq1 = SchemeSmob::verify_handle_list(list1, scheme_name, 2);

					// Third argument is a possibly empty list of Handles
					SCM list2 = scm_caddr(args);
					HandleSeq seq2 = SchemeSmob::verify_handle_list(list2, scheme_name, 3);

					Handle rh = (that->*method.h_sqq)(str, seq1, seq2);
					rc = SchemeSmob::handle_to_scm(rh);
					break;
				}
				case Q_HTI:
				{
					// First arg is a handle
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name, 1);
					
					// Second arg is a type
					Type t = SchemeSmob::verify_atom_type(scm_cadr(args), scheme_name, 2);

					// Third arg is an int
					int i = SchemeSmob::verify_int(scm_caddr(args), scheme_name, 3);

					HandleSeq rHS = (that->*method.q_hti)(h,t,i);
					HandleSeq::iterator it = rHS.begin();
					if (it != rHS.end())
						rc = scm_list_1(SchemeSmob::handle_to_scm(*it));
					++it;
					for ( ; it != rHS.end(); ++it)
					{
						rc = scm_cons(SchemeSmob::handle_to_scm(*it), rc);
					}
					break;
				}
				case Q_HTIB:
				{
					// First arg is a handle
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name, 1);
					
					// Second arg is a type
					Type t = SchemeSmob::verify_atom_type(scm_cadr(args), scheme_name, 2);

					// Third arg is an int
					int i = SchemeSmob::verify_int(scm_caddr(args), scheme_name, 3);
					
					//Fourth arg is a bool
					bool b = scm_to_bool(scm_cadddr(args));

					HandleSeq rHS = (that->*method.q_htib)(h,t,i,b);
					HandleSeq::iterator it = rHS.begin();
					if (it != rHS.end())
						rc = scm_list_1(SchemeSmob::handle_to_scm(*it));
					++it;
					for ( ; it != rHS.end(); ++it)
					{
						rc = scm_cons(SchemeSmob::handle_to_scm(*it), rc);
					}
					break;
				}
				case S_S:
				{
					// First argument is a string
					std::string str = SchemeSmob::verify_string(scm_car(args), scheme_name, 1);

					const std::string &rs = (that->*method.s_s)(str);
					rc = scm_from_locale_string(rs.c_str());
					break;
				}
				case V_H:
				{
					Handle h = SchemeSmob::verify_handle(scm_car(args), scheme_name);
					(that->*method.h_h)(h);
					break;
				}
				case V_T:
				{
					Type t = SchemeSmob::verify_atom_type(scm_car(args), scheme_name, 1);
					(that->*method.v_t)(t);
					break;
				}
				case V_TI:
				{
					SCM input = scm_car(args);
					//Assuming that the type is input as a string or symbol, eg
					//(f 'SimilarityLink) or (f "SimilarityLink")
					if (scm_is_true(scm_symbol_p(input)))
						input = scm_symbol_to_string(input);
					
					char *lstr = scm_to_locale_string(input);
					Type t = classserver().getType(lstr);
					free(lstr);
					
					int i = scm_to_int(scm_cadr(args));
					
					(that->*method.v_ti)(t, i);
					break;
				}
				case V_TIDI:
				{
					SCM input = scm_car(args);
					//Assuming that the type is input as a string or symbol, eg
					//(f 'SimilarityLink) or (f "SimilarityLink")
					if (scm_is_true(scm_symbol_p(input)))
						input = scm_symbol_to_string(input);
					
					char *lstr = scm_to_locale_string(input);
					Type t = classserver().getType(lstr);
					free(lstr);
					
					int i = scm_to_int(scm_cadr(args));
					double d = scm_to_double(scm_caddr(args));
					int i2 = scm_to_int(scm_cadddr(args));
					
					(that->*method.v_tidi)(t, i, d, i2);
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
	
#define DECLARE_CONSTR_3(SIG, LSIG, RET_TYPE, ARG1_TYPE, ARG2_TYPE, ARG3_TYPE) \
	SchemePrimitive(const char *name, RET_TYPE (T::*cb)(ARG1_TYPE, ARG2_TYPE, ARG3_TYPE), T *data) \
	{ \
		that = data; \
		method.LSIG = cb; \
		scheme_name = name; \
		signature = SIG; \
		do_register(name, 3); /* cb has 3 args */ \
	}
#define DECLARE_CONSTR_4(SIG, LSIG, RET_TYPE, ARG1_TYPE, ARG2_TYPE, ARG3_TYPE, ARG4_TYPE) \
	SchemePrimitive(const char *name, RET_TYPE (T::*cb)(ARG1_TYPE, ARG2_TYPE, ARG3_TYPE, ARG4_TYPE), T *data) \
	{ \
		that = data; \
		method.LSIG = cb; \
		scheme_name = name; \
		signature = SIG; \
		do_register(name, 4); /* cb has 4 args */ \
	}

		// Declare and define the constructors for this class. They all have
		// the same basic form, except for the types.
		DECLARE_CONSTR_2(B_HI, b_hi, bool, Handle, int)
		DECLARE_CONSTR_3(D_HHT, d_hht, double, Handle, Handle, Type)
		DECLARE_CONSTR_4(D_HHTB, d_hhtb, double, Handle, Handle, Type, bool)
		DECLARE_CONSTR_1(H_H,  h_h,  Handle, Handle)
		DECLARE_CONSTR_2(H_HI, h_hi, Handle, Handle, int)
		DECLARE_CONSTR_2(H_SQ, h_sq, Handle, const std::string&, const HandleSeq&)
		DECLARE_CONSTR_3(H_SQQ, h_sqq, Handle, const std::string&, const HandleSeq&, const HandleSeq&)
		DECLARE_CONSTR_3(Q_HTI, q_hti, HandleSeq, Handle, Type, int)
		DECLARE_CONSTR_4(Q_HTIB, q_htib, HandleSeq, Handle, Type, int, bool)
		DECLARE_CONSTR_1(S_S,  s_s,  const std::string&, const std::string&)
		DECLARE_CONSTR_1(V_H, v_h, void, Handle)
		DECLARE_CONSTR_1(V_T, v_t, void, Type)
		DECLARE_CONSTR_2(V_TI, v_ti, void, Type, int)
		DECLARE_CONSTR_4(V_TIDI, v_tidi, void, Type, int, double, int)

		// Below is DECLARE_CONSTR_0(V_V, v_v, void*, void);
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

#define DECLARE_DECLARE_3(RET,ARG1,ARG2,ARG3) \
template<class T> \
inline void define_scheme_primitive(const char *name, RET (T::*cb)(ARG1,ARG2,ARG3), T *data) \
{ \
	/* Note: this is freed automatically by scheme garbage collection */ \
	/* when it is no longer needed. */ \
	new SchemePrimitive<T>(name, cb, data); \
}
#define DECLARE_DECLARE_4(RET,ARG1,ARG2,ARG3,ARG4) \
template<class T> \
inline void define_scheme_primitive(const char *name, RET (T::*cb)(ARG1,ARG2,ARG3,ARG4), T *data) \
{ \
	/* Note: this is freed automatically by scheme garbage collection */ \
	/* when it is no longer needed. */ \
	new SchemePrimitive<T>(name, cb, data); \
}

DECLARE_DECLARE_1(Handle, Handle)
DECLARE_DECLARE_1(const std::string&, const std::string&)
DECLARE_DECLARE_1(void, Handle)
DECLARE_DECLARE_1(void, Type)
DECLARE_DECLARE_1(void, void)
DECLARE_DECLARE_2(bool, Handle, int)
DECLARE_DECLARE_2(Handle, Handle, int)
DECLARE_DECLARE_2(Handle, const std::string&, const HandleSeq&)
DECLARE_DECLARE_2(void, Type, int)
DECLARE_DECLARE_3(double, Handle, Handle, Type)
DECLARE_DECLARE_3(Handle, const std::string&, const HandleSeq&, const HandleSeq&)
DECLARE_DECLARE_3(HandleSeq, Handle, Type, int)
DECLARE_DECLARE_4(double, Handle, Handle, Type, bool)
DECLARE_DECLARE_4(void, Type, int, double, int)
DECLARE_DECLARE_4(HandleSeq, Handle, Type, int, bool)

/** @}*/
}

#endif // _OPENCOG_SCHEME_PRIMITIVE_H

#endif // HAVE_GUILE

