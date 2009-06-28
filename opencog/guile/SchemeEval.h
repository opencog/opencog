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
#include <pthread.h>
#include <libguile.h>
#include <opencog/atomspace/types.h>

namespace opencog {

class SchemeEval
{
	private:
		// Initialization stuff
		void init(void);
		static void * c_wrap_init(void *);
		void per_thread_init(void);
		void thread_lock(void);
		void thread_unlock(void);

		// destructor stuff
		void finish(void);
		static void * c_wrap_finish(void *);

		// Things related to evaluation
		std::string do_eval(const std::string &);
		static void * c_wrap_eval(void *);
		const std::string *pexpr;
		std::string answer;

		std::string input_line;
		bool pending_input;

		// Handle apply
		Handle do_apply(const std::string&, Handle args);
		Handle hargs;
		static void * c_wrap_apply(void *);

		// straight-up evaluation
		SCM do_scm_eval(SCM);
		static SCM wrap_scm_eval(void *);

		// Error handling stuff
		SCM error_string_port;
		SCM captured_stack;
		static SCM preunwind_handler_wrapper(void *, SCM, SCM);
		static SCM catch_handler_wrapper(void *, SCM, SCM);
		SCM preunwind_handler(SCM, SCM);
		SCM catch_handler(SCM, SCM);
		bool caught_error;

		// printing of basic types
		static std::string prt(SCM);

		// output port
		SCM outport;
		SCM saved_outport;

	public:
		SchemeEval(void);
		~SchemeEval();
		std::string eval(const std::string &);
		Handle apply(const std::string&, Handle args);

		bool input_pending(void);
		void clear_pending(void);
		bool eval_error(void);
};

}

#else /* HAVE_GUILE */

#include <opencog/atomspace/types.h>

namespace opencog {

class SchemeEval
{
	public:
		std::string eval(const std::string &) { return ""; }
		Handle apply(const std::string&, Handle args) {
			return Handle::UNDEFINED; }

		bool input_pending(void) { return false; }
		void clear_pending(void) {}
		bool eval_error(void) { return false; }
};

}
#endif/* HAVE_GUILE */
#endif /* OPENCOG_SCHEME_EVAL_H */
