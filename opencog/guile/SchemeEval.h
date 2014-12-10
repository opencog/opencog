/*
 * SchemeEval.h
 *
 * Scheme expression evaluator for OpenCog
 * Copyright (c) 2008, 2014 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SCHEME_EVAL_H
#define OPENCOG_SCHEME_EVAL_H
#ifdef HAVE_GUILE

#include <condition_variable>
#include <mutex>
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
 *
 * The evaluator provides two modes of usage: synchronous and
 * asynchronous.  The synchronous API is provided by the following
 * methods: eval(), eval_h(), apply(), and apply_generic(). The last
 * three are special-purpose wrappers around the first, returning
 * handles, and applying functions to argument lists.
 *
 * The eval() method returns a string, holding any output that was
 * printed during evaluation. e.g. output printed using the scheme
 * 'display' function. If the code to be evaluated is long-running,
 * then nothing can be returned until the evaluation completes. This
 * presents a problem when the thing to be evaluated is perhaps an
 * infinite loop.
 *
 * Thus, the asynchronous interface is provided. This is implemented
 * with three methods: begin_eval(), eval_expr(), and poll_result().
 * If the expression that needs to be evaluated is long-running, or
 * even if it is an infinite loop, yet which periodically prints, then
 * it can be run in one thread, and the print output can be collected in
 * another thread, with poll_result().  The poll_result() method will
 * block until there is printed output. It can be called repeatedly.
 * When evaluation is completed, it will return any pending printed
 * output, and subsequent calls will return the empty string
 * immediately, without any further blocking.  Be sure to call
 * begin_eval() first, to set things up.
 *
 * The synchronous implementation is built on top of the async one,
 * and runs entirely within the same thread; see the code below; it
 * should show that:
 *
 *      std::string eval(const std::string& expr)
 *         { begin_eval(); eval_expr(expr); return poll_result(); }
 *
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

		// Things related to (async) shell-evaluation
		void do_eval(const std::string &);
		std::string do_poll_result();
		static void * c_wrap_eval(void *);
		static void * c_wrap_poll(void *);
		const std::string *pexpr;
		std::string answer;
		SCM _rc;
		bool _eval_done;
		bool _poll_done;
		std::mutex _poll_mtx;
		std::condition_variable _wait_done;
		std::string poll_port();
		SCM _pipe;
		int _pipeno;

		// Straight-up evaluation
		static SCM thunk_scm_eval(void *);
		SCM do_scm_eval(SCM);
		SCM do_scm_eval_str(const std::string &);
		static void * c_wrap_eval_h(void *);

		// Handle apply
		Handle do_apply(const std::string& func, Handle varargs);
		SCM do_apply_scm(const std::string& func, Handle varargs);
		Handle hargs;
		static void * c_wrap_apply(void *);
		static void * c_wrap_apply_scm(void *);

		// Error handling stuff
		SCM error_string;
		void set_error_string(SCM);
		SCM captured_stack;
		void set_captured_stack(SCM);
		static SCM preunwind_handler_wrapper(void *, SCM, SCM);
		static SCM catch_handler_wrapper(void *, SCM, SCM);
		SCM preunwind_handler(SCM, SCM);
		SCM catch_handler(SCM, SCM);

		// Printing of basic types
		static std::string prt(SCM);

		// Output port
		SCM _outport;
		SCM _saved_outport;
		bool _in_shell;
		void drain_output();

		AtomSpace* atomspace;
		int _gc_ctr;

	public:
		SchemeEval(AtomSpace*);
		~SchemeEval();

		// The async-output interface.
		void begin_eval();
		void eval_expr(const std::string&);
		std::string poll_result();

		// The synchronous-output interfaces.
		std::string eval(const std::string& expr)
			{ begin_eval(); eval_expr(expr); return poll_result(); }
		std::string eval(const std::stringstream& ss)
			{ return eval(ss.str()); }

		Handle eval_h(const std::string&);
		Handle eval_h(const std::stringstream& ss) { return eval_h(ss.str()); }

		Handle apply(const std::string& func, Handle varargs);
		std::string apply_generic(const std::string& func, Handle varargs);
};

/** @}*/
}

#endif/* HAVE_GUILE */

#endif /* OPENCOG_SCHEME_EVAL_H */
