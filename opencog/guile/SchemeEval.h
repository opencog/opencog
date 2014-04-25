/*
 * SchemeEval.h
 *
 * Simple scheme expression evaluator
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SCHEME_EVAL_H
#define OPENCOG_SCHEME_EVAL_H
#ifdef HAVE_GUILE

#include <string>
#include <sstream>
#include <pthread.h>
#include <libguile.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/shell/GenericEval.h>
#include <opencog/util/exceptions.h>

namespace opencog {
/** \addtogroup grp_smob
 *  @{
 */

class AtomSpace;

class SchemeEval : public GenericEval
{
	private:
		// Initialization stuff
		void init(void);
		static void * c_wrap_init(void *);
		void per_thread_init(void);
		void thread_lock(void);
		void thread_unlock(void);

		// Destructor stuff
		void finish(void);
		static void * c_wrap_finish(void *);

		// Things related to evaluation
		std::string do_eval(const std::string &);
		static void * c_wrap_eval(void *);
		static void * c_wrap_eval_h(void *);
		const std::string *pexpr;
		std::string answer;

		// Straight-up evaluation
		static SCM wrap_scm_eval(void *);
		SCM do_scm_eval(SCM);
		SCM do_scm_eval_str(const std::string &);

		// Handle apply
		Handle do_apply(const std::string& func, Handle varargs);
		SCM do_apply_scm(const std::string& func, Handle varargs);
		Handle hargs;
		static void * c_wrap_apply(void *);
		static void * c_wrap_apply_scm(void *);

		// Error handling stuff
		SCM error_string;
		SCM captured_stack;
		static SCM preunwind_handler_wrapper(void *, SCM, SCM);
		static SCM catch_handler_wrapper(void *, SCM, SCM);
		SCM preunwind_handler(SCM, SCM);
		SCM catch_handler(SCM, SCM);

		// Printing of basic types
		static std::string prt(SCM);

		// Output port
		SCM outport;
		SCM saved_outport;
		bool in_shell;
		AtomSpace* atomspace;
		SCM the_environment;

	public:
		SchemeEval(AtomSpace* = NULL);
		~SchemeEval();

		std::string eval(const std::string&);
		std::string eval(const std::stringstream& ss) { return eval(ss.str()); }

		Handle eval_h(const std::string&);
		Handle eval_h(const std::stringstream& ss) { return eval_h(ss.str()); }

		Handle apply(const std::string& func, Handle varargs);
		std::string apply_generic(const std::string& func, Handle varargs);
};

/** @}*/
}

#endif/* HAVE_GUILE */

#endif /* OPENCOG_SCHEME_EVAL_H */
