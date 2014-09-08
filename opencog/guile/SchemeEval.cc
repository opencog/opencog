/**
 * SchemeEval.cc
 *
 * Scheme evaluator.  Given strings in scheme, evaluates them with
 * the appropriate Atomspace, etc.
 *
 * Copyright (c) 2008, 2014 Linas Vepstas
 */

#ifdef HAVE_GUILE

#include <unistd.h>
#include <fcntl.h>

#include <libguile.h>
#include <libguile/backtrace.h>
#include <libguile/debug.h>
#ifndef HAVE_GUILE2
  #include <libguile/lang.h>
#endif
#include <pthread.h>

#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>

#include "SchemeEval.h"
#include "SchemePrimitive.h"
#include "SchemeSmob.h"

using namespace opencog;

/**
 * This init is called once for every time that this class
 * is instantiated -- i.e. it is a per-instance initializer.
 */
void SchemeEval::init(void)
{
	SchemeSmob::init();
	PrimitiveEnviron::init();

	// Output ports for side-effects.
	_saved_outport = scm_current_output_port();
	_saved_outport = scm_gc_protect_object(_saved_outport);

	SCM pair = scm_pipe();
	_pipe = scm_car(pair);
	_pipe = scm_gc_protect_object(_pipe);
	_pipeno = scm_to_int(scm_fileno(_pipe));
	_outport = scm_cdr(pair);
	_outport = scm_gc_protect_object(_outport);
	scm_setvbuf(_outport, scm_from_int (_IONBF), SCM_UNDEFINED);

	// We want non-blocking reads.
	int flags = fcntl(_pipeno, F_GETFL, 0);
	if (flags < 0) flags = 0;
	fcntl(_pipeno, F_SETFL, flags | O_NONBLOCK);

	scm_set_current_output_port(_outport);

	_in_shell = false;

	// User error and crash management
	error_string = SCM_EOL;
	error_string = scm_gc_protect_object(error_string);

	captured_stack = SCM_BOOL_F;
	captured_stack = scm_gc_protect_object(captured_stack);

	pexpr = NULL;
	_eval_done = false;
	_poll_done = false;
}

/// Discard all chars in the outport.
void SchemeEval::drain_output(void)
{
	// read and discard.
	poll_port();
}

void * SchemeEval::c_wrap_init(void *p)
{
	SchemeEval *self = (SchemeEval *) p;
	self->init();
	return self;
}

void SchemeEval::finish(void)
{
	// Restore the previous outport (if its still alive)
	if (scm_is_false(scm_port_closed_p(_saved_outport)))
		scm_set_current_output_port(_saved_outport);
	scm_gc_unprotect_object(_saved_outport);

	scm_close_port(_outport);
	scm_gc_unprotect_object(_outport);

	scm_close_port(_pipe);
	scm_gc_unprotect_object(_pipe);

	scm_gc_unprotect_object(error_string);
	scm_gc_unprotect_object(captured_stack);
}

void * SchemeEval::c_wrap_finish(void *p)
{
	SchemeEval *self = (SchemeEval *) p;
	self->finish();
	return self;
}

// The following two routines are needed to avoid bad garbage collection
// of anything we've kept in the object.
void SchemeEval::set_captured_stack(SCM newstack)
{
	// protect before unprotecting, to avoid multi-threaded races.
	SCM oldstack = captured_stack;
	captured_stack = scm_gc_protect_object(newstack);
	scm_gc_unprotect_object(oldstack);
}

void SchemeEval::set_error_string(SCM newerror)
{
	SCM olderror = error_string;
	error_string = scm_gc_protect_object(newerror);
	scm_gc_unprotect_object(olderror);
}

static pthread_once_t eval_init_once = PTHREAD_ONCE_INIT;
static pthread_key_t tid_key = 0;

#define WORK_AROUND_GUILE_185_BUG
#ifdef WORK_AROUND_GUILE_185_BUG
/* There's a bug in guile-1.8.5, where the second and subsequent
 * threads run in guile mode with a bogus/broken current-module.
 * This cannot be worked around by anything as simple as saying
 * "(set-current-module the-root-module)" because dynwind undoes
 * any module-setting that we do.
 *
 * So we work around it here, by explicitly setting the module
 * outside of a dynwind context.
 */
static SCM guile_user_module;

static void * do_bogus_scm(void *p)
{
	scm_c_eval_string ("(+ 2 2)\n");
	return p;
}
#endif /* WORK_AROUND_GUILE_185_BUG */

#define WORK_AROUND_GUILE_THREADING_BUG
#ifdef WORK_AROUND_GUILE_THREADING_BUG
/* There are bugs in guile-1.8.6 and earlier that prevent proper
 * multi-threaded operation. Currently, the most serious of these is
 * a parallel-define bug, documented in
 * https://savannah.gnu.org/bugs/index.php?24867
 *
 * Until that bug is fixed and released, this work-around is needed.
 * The work-around serializes all guile-mode thread execution, by
 * means of a mutex lock.
 *
 * As of December 2013, the bug still seems to be there: the test
 * case provided in the bug report crashes, when linked against
 * guile-2.0.5 and gc-7.1 from Ubuntu Precise.
 *
 * Its claimed that the bug only happens for top-level defines.
 * Thus, in principle, threading should be OK after all scripts have
 * been loaded.
 */
static pthread_mutex_t serialize_lock;
static pthread_key_t ser_key = 0;
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

static void first_time_only(void)
{
	pthread_key_create(&tid_key, NULL);
	pthread_setspecific(tid_key, (const void *) 0x42);
#ifdef WORK_AROUND_GUILE_THREADING_BUG
	pthread_mutex_init(&serialize_lock, NULL);
	pthread_key_create(&ser_key, NULL);
	pthread_setspecific(ser_key, (const void *) 0x0);
#endif /* WORK_AROUND_GUILE_THREADING_BUG */
#ifdef WORK_AROUND_GUILE_185_BUG
	scm_with_guile(do_bogus_scm, NULL);
	guile_user_module = scm_current_module();
#endif /* WORK_AROUND_GUILE_185_BUG */
}

#ifdef WORK_AROUND_GUILE_THREADING_BUG

/**
 * This lock primitive allow nested locks within one thread,
 * but prevents concurrent threads from running.
 */
void SchemeEval::thread_lock(void)
{
	long cnt = (long) pthread_getspecific(ser_key);
	if (0 >= cnt)
	{
		pthread_mutex_lock(&serialize_lock);
	}
	cnt ++;
	pthread_setspecific(ser_key, (const void *) cnt);
}

void SchemeEval::thread_unlock(void)
{
	long cnt = (long) pthread_getspecific(ser_key);
	cnt --;
	pthread_setspecific(ser_key, (const void *) cnt);
	if (0 >= cnt)
	{
		pthread_mutex_unlock(&serialize_lock);
	}
}
#endif

SchemeEval::SchemeEval(AtomSpace* as)
{
	pthread_once(&eval_init_once, first_time_only);

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_lock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */
	atomspace = as;

	scm_with_guile(c_wrap_init, this);

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_unlock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

}

/* This should be called once for every new thread. */
void SchemeEval::per_thread_init(void)
{
	/* Avoid more than one call per thread. */
	if (((void *) 0x2) == pthread_getspecific(tid_key)) return;
	pthread_setspecific(tid_key, (const void *) 0x2);

#ifdef WORK_AROUND_GUILE_185_BUG
	scm_set_current_module(guile_user_module);
#endif /* WORK_AROUND_GUILE_185_BUG */

	// Guile implements the current port as a fluid on each thread.
	// So, for every new thread, we need to set this.
	scm_set_current_output_port(_outport);
}

SchemeEval::~SchemeEval()
{
	scm_with_guile(c_wrap_finish, this);
}

/* ============================================================== */

std::string SchemeEval::prt(SCM node)
{
	if (SCM_SMOB_PREDICATE(SchemeSmob::cog_uuid_tag, node))
	{
		return SchemeSmob::uuid_to_string(node);
	}
	else if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, node))
	{
		return SchemeSmob::misc_to_string(node);
	}
	else if (scm_is_eq(node, SCM_UNSPECIFIED))
	{
		return "";
	}
	else
	{
		// Let SCM display do the rest of the work.
		SCM port = scm_open_output_string();
		scm_display (node, port);
		SCM rc = scm_get_output_string(port);
		char * str = scm_to_locale_string(rc);
		std::string rv = str;
		free(str);
		scm_close_port(port);
		return rv;
	}

	return "";
}

/* ============================================================== */

SCM SchemeEval::preunwind_handler_wrapper (void *data, SCM tag, SCM throw_args)
{
	SchemeEval *ss = (SchemeEval *) data;
	return ss->preunwind_handler(tag, throw_args);
}

SCM SchemeEval::catch_handler_wrapper (void *data, SCM tag, SCM throw_args)
{
	SchemeEval *ss = (SchemeEval *) data;
	return ss->catch_handler(tag, throw_args);
}

SCM SchemeEval::preunwind_handler (SCM tag, SCM throw_args)
{
	// We can only record the stack before it is unwound.
	// The normal catch handler body runs only *after* the stack
	// has been unwound.
	set_captured_stack(scm_make_stack(SCM_BOOL_T, SCM_EOL));
	return SCM_EOL;
}

SCM SchemeEval::catch_handler (SCM tag, SCM throw_args)
{
	// Check for read error. If a read error, then wait for user to correct it.
	SCM re = scm_symbol_to_string(tag);
	char * restr = scm_to_locale_string(re);
	_pending_input = false;

	if (0 == strcmp(restr, "read-error"))
	{
		_pending_input = true;
		free(restr);
		return SCM_EOL;
	}

	// Check for a simple flow-control directive: i.e. just return to
	// the C code from anywhere within the scheme code.
	if (0 == strcmp(restr, "cog-yield"))
	{
		free(restr);
		return SCM_CAR(throw_args);
	}

	// If it's not a read error, and it's not flow-control,
	// then its a regular error; report it.
	_caught_error = true;

	/* get string port into which we write the error message and stack. */
	SCM port = scm_open_output_string();

	if (scm_is_true(scm_list_p(throw_args)) && (scm_ilength(throw_args) >= 1))
	{
		long nargs = scm_ilength(throw_args);
		SCM subr	= SCM_CAR (throw_args);
		SCM message = SCM_EOL;
		if (nargs >= 2)
			message = SCM_CADR (throw_args);
		SCM parts   = SCM_EOL;
		if (nargs >= 3)
			parts   = SCM_CADDR (throw_args);
		SCM rest	= SCM_EOL;
		if (nargs >= 4)
			rest	= SCM_CADDDR (throw_args);

		if (scm_is_true (captured_stack))
		{
			SCM highlights;

			if (scm_is_eq (tag, scm_arg_type_key) ||
				scm_is_eq (tag, scm_out_of_range_key))
				highlights = rest;
			else
				highlights = SCM_EOL;

			scm_puts ("Backtrace:\n", port);
			scm_display_backtrace_with_highlights (captured_stack, port,
			                                       SCM_BOOL_F, SCM_BOOL_F,
			                                       highlights);
			scm_newline (port);
		}
#ifdef HAVE_GUILE2
		if (SCM_STACK_LENGTH (captured_stack))
			set_captured_stack(scm_stack_ref (captured_stack, SCM_INUM0));
#endif
		scm_display_error (captured_stack, port, subr, message, parts, rest);
	}
	else
	{
		scm_puts ("ERROR: throw args are unexpectedly short!\n", port);
	}
	scm_puts("ABORT: ", port);
	scm_puts(restr, port);
	free(restr);

	set_error_string(scm_get_output_string(port));
	scm_close_port(port);
	return SCM_BOOL_F;
}

/* ============================================================== */
/**
 * Evaluate a scheme expression.
 *
 * This evaluator is tailored for being invoked by a shell, in several
 * different ways:
 *
 * 1) It buffers up incomplete, line-by-line input, until there's
 *	been enough input received to evaluate without error.
 * 2) It catches errors, and prints the catch in a reasonably nicely
 *	formatted way.
 * 3) It converts any returned scheme expressions to a string, for easy
 *	printing.
 * 4) It concatenates any data sent to the scheme output port (e.g.
 *	printed output from the scheme (display) function) to the returned
 *	string.
 *
 * An "unforgiving" evaluator, with none of these amenities, can be
 * found in eval_h(), below.
 */
void SchemeEval::eval_expr(const std::string &expr)
{
	pexpr = &expr;

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_lock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

	_in_shell = true;
	scm_with_guile(c_wrap_eval, this);
	_in_shell = false;

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_unlock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */
}

void* SchemeEval::c_wrap_poll(void* p)
{
	SchemeEval* self = (SchemeEval*) p;
	self->answer = self->do_poll_result();
	return p;
}

std::string SchemeEval::poll_result()
{
	scm_with_guile(c_wrap_poll, this);
	return answer;
}

void* SchemeEval::c_wrap_eval(void* p)
{
	SchemeEval* self = (SchemeEval*) p;

	// Normally, neither of these are ever null.
	// But sometimes, a heavily loaded server can crash here.
	// Trying to figure out why ...
	OC_ASSERT(self, "c_wrap_eval got null pointer!");
	OC_ASSERT(self->pexpr, "c_wrap_eval got null expression!");

	self->do_eval(*(self->pexpr));
	return self;
}

/**
 * do_eval -- evaluate a scheme expression string.
 * This implements the working guts of the shell-freindly evaluator.
 *
 * This method *must* be called in guile mode, in order for garbage
 * collection, etc. to work correctly!
 */
void SchemeEval::do_eval(const std::string &expr)
{
	per_thread_init();

	// Set global atomspace variable in the execution environment.
	SchemeSmob::ss_set_env_as(atomspace);

	_input_line += expr;

	_caught_error = false;
	_pending_input = false;
	set_captured_stack(SCM_BOOL_F);
	_rc = scm_c_catch (SCM_BOOL_T,
	                      (scm_t_catch_body) scm_c_eval_string,
	                      (void *) _input_line.c_str(),
	                      SchemeEval::catch_handler_wrapper, this,
	                      SchemeEval::preunwind_handler_wrapper, this);

	atomspace = SchemeSmob::ss_get_env_as("do_eval");
	_eval_done = true;
	_wait_done.notify_all();
}

void SchemeEval::begin_eval()
{
	_eval_done = false;
	_poll_done = false;
	_rc = SCM_EOL;
}

/// Read one end of a pipe. The other end of the pipe is attached to
/// guile's default output port.  We use standard posix to read, as
/// that will be faster than mucking with guile's one-char-at-a-time
/// API.  The below assumes that the pipe is non-blcoking; if it blocks,
/// then things might get wonky.  The below is also very simple; it does
/// not check for any standard unix errors, closed pipes, etc. I think
/// that simplicity is just fine, here.
std::string SchemeEval::poll_port()
{
	std::string rv;
#define BUFSZ 1000
	char buff[BUFSZ];
	while (1)
	{
		int nr = read(_pipeno, buff, BUFSZ-1);
		if (-1 == nr) return rv;
		buff[nr] = 0x0;
		rv += buff;
	}
	return rv;
}

/// Get output from evaluator, if any; block otherwise.
///
/// This method is meant to be called from a different thread than
/// the one that the scheme evaluator is running in.  It will try
/// to get any pending output, and will return that.  If there's no
/// pending output, then it will block until (1) there's output, or
/// (2) the evaluation has completed, in which case, it will return all
/// remaining output generated by the eval.
///
/// The entire goal of this function is to allow the use of the guile
/// 'display' can other stdio output calls from the interactive
/// environment, even if the actual scheme eval is very long-running.
/// Without this, what would happen is that all the output would get
/// buffered up, until the eval() finished, and then all this output
/// would get dumped.  For anything that runs more than a few seconds
/// this would be confusing and undesirable.  So, this method allows
/// some other thread to see if eval() is generating output, and, if
/// it is, to print it to stdout.
std::string SchemeEval::do_poll_result()
{
	per_thread_init();
	if (_poll_done) return "";

	if (not _eval_done)
	{
		// We don't have a real need to lock anything here; we're just
		// using this as a hack, so that the condition variable will
		// wake us up periodically.  The goal here is to block when
		// there's no output to be reported.
		//
		// Hmmm. I guess we could just block on reading the pipe...
		// unless the eval did not produce any output, in which case
		// I guess we could ... close and re-open the pipe? Somehow
		// unblock it?  I dunno. The below curently works, and I'm
		// loosing interest just right now.
		std::unique_lock<std::mutex> lck(_poll_mtx);
		while (not _eval_done)
		{
			_wait_done.wait_for(lck, std::chrono::milliseconds(300));
			std::string rv = poll_port();
			if (0 < rv.size()) return rv;
		}
	}
	// If we are here, then evaluation is done. Check the various
	// evalution result flags, etc.
	_poll_done = true;

	/* An error is thrown if the input expression is incomplete,
	 * in which case the error handler sets the _pending_input flag
	 * to true. */
	if (_pending_input)
	{
		return "";
	}
	_pending_input = false;
	_input_line = "";

	if (_caught_error)
	{
		std::string rv = poll_port();

		char * str = scm_to_locale_string(error_string);
		rv += str;
		free(str);
		set_error_string(SCM_EOL);
		set_captured_stack(SCM_BOOL_F);

		rv += "\n";
		return rv;
	}
	else
	{
		// First, we get the contents of the output port,
		// and pass that on.
		std::string rv = poll_port();

		// Next, we append the "interpreter" output
		rv += prt(_rc);
		rv += "\n";
		return rv;
	}
	return "#<Error: Unreachable statement reached>";
}

/* ============================================================== */

SCM SchemeEval::thunk_scm_eval(void* expr)
{
	SCM sexpr = (SCM) expr;
	return scm_eval(sexpr, scm_interaction_environment());
}

/**
 * do_scm_eval -- evaluate a scheme expression
 *
 * Similar to do_eval(), with several important differences:
 * 1) The argument must be an SCM expression.
 * 2) No shell-freindly string and output management is performed.
 * 3) Evaluation errors are logged to the log file.
 *
 * This method *must* be called in guile mode, in order for garbage
 * collection, etc. to work correctly!
 */
SCM SchemeEval::do_scm_eval(SCM sexpr)
{
	per_thread_init();

	// Set global atomspace variable in the execution environment.
	SchemeSmob::ss_set_env_as(atomspace);

	_caught_error = false;
	set_captured_stack(SCM_BOOL_F);
	SCM rc = scm_c_catch (SCM_BOOL_T,
	                 SchemeEval::thunk_scm_eval, (void *) sexpr,
	                 SchemeEval::catch_handler_wrapper, this,
	                 SchemeEval::preunwind_handler_wrapper, this);

	_eval_done = true;
	if (_caught_error)
	{
		char * str = scm_to_locale_string(error_string);
		// Don't blank out the error string yet.... we need it later.
		// (probably because someone called cog-bind with an ExecutionLink
		// in it with a bad scheme schema node.)
		// error_string = SCM_EOL;
		set_captured_stack(SCM_BOOL_F);

		drain_output();

		// Unlike errors seen on the interpreter, log these to the logger.
		// That's because these errors will be predominantly script
		// errors that are otherwise invisible to the user/developer.
		Logger::Level save = logger().getBackTraceLevel();
		logger().setBackTraceLevel(Logger::NONE);
		logger().error("%s: guile error was: %s\nFailing expression was %s",
		               __FUNCTION__, str, prt(sexpr).c_str());
		logger().setBackTraceLevel(save);

		free(str);
		return SCM_EOL;
	}

	atomspace = SchemeSmob::ss_get_env_as("do_scm_eval");

	// Get the contents of the output port, and log it
	if (logger().isInfoEnabled())
	{
		std::string str(poll_port());
		if (0 < str.size())
		{
			logger().info("%s: Output: %s\n"
			              "Was generated by expr: %s\n",
			              __FUNCTION__, str.c_str(), prt(sexpr).c_str());
		}
	}

	// If we are not in a shell context, truncate the output, because
	// it will never ever be displayed. (i.e. don't overflow the output
	// buffers.) If we are in_shell, then we are here probably because
	// user typed something that caused some ExecutionLink to call some
	// scheme snippet.  Display that.
	if (not _in_shell)
		drain_output();

	return rc;
}

/* ============================================================== */
/**
 * Evaluate a string containing a scheme expression, returning a Handle.
 * If an evaluation error occurs, then the error is logged to the log
 * file, and Handle::UNDEFINED is returned.  If the result of evaluation
 * is not a handle, no error is logged, but Handle::UNDEFINED is still
 * returned. Otherwise, if the result of evaluation is a handle, that
 * handle is returned.
 */
Handle SchemeEval::eval_h(const std::string &expr)
{
	pexpr = &expr;

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_lock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

	scm_with_guile(c_wrap_eval_h, this);

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_unlock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

	return hargs;
}

void * SchemeEval::c_wrap_eval_h(void * p)
{
	SchemeEval *self = (SchemeEval *) p;
	SCM rc = self->do_scm_eval_str(*(self->pexpr));
	self->hargs = SchemeSmob::scm_to_handle(rc);
	return self;
}

/**
 * do_scm_eval_str -- evaluate a scheme expression
 *
 * Similar to do_scm_eval(), but several important differences:
 * 1) The arugment must be a string containing valid scheme,
 * 2) No shell-freindly string and output management is performed,
 * 3) Evaluation errors are logged to the log file.
 *
 * This method *must* be called in guile mode, in order for garbage
 * collection, etc. to work correctly!
 */
SCM SchemeEval::do_scm_eval_str(const std::string &expr)
{
	per_thread_init();

	// Set global atomspace variable in the execution environment.
	SchemeSmob::ss_set_env_as(atomspace);

	_caught_error = false;
	set_captured_stack(SCM_BOOL_F);
	SCM rc = scm_c_catch (SCM_BOOL_T,
	                      (scm_t_catch_body) scm_c_eval_string,
	                      (void *) expr.c_str(),
	                      SchemeEval::catch_handler_wrapper, this,
	                      SchemeEval::preunwind_handler_wrapper, this);

	_eval_done = true;
	if (_caught_error)
	{
		char * str = scm_to_locale_string(error_string);
		set_error_string(SCM_EOL);
		set_captured_stack(SCM_BOOL_F);

		drain_output();

		// Unlike errors seen on the interpreter, log these to the logger.
		// That's because these errors will be predominantly script
		// errors that are otherwise invisible to the user/developer.
		Logger::Level save = logger().getBackTraceLevel();
		logger().setBackTraceLevel(Logger::NONE);
		logger().error("%s: guile error was: %s\nFailing expression was %s",
		               __FUNCTION__, str, expr.c_str());
		logger().setBackTraceLevel(save);

		free(str);
		return SCM_EOL;
	}

	atomspace = SchemeSmob::ss_get_env_as("do_scm_eval_str");

	// Get the contents of the output port, and log it
	if (logger().isInfoEnabled())
	{
		std::string str(poll_port());
		if (0 < str.size())
		{
			logger().info("%s: Output: %s\nWas generated by expr: %s\n",
			              __FUNCTION__, str.c_str(), expr.c_str());
		}
	}
	drain_output();

	return rc;
}

/* ============================================================== */
/**
 * apply -- apply named function func to arguments in ListLink
 * It is assumed that varargs is a ListLink, containing a list of
 * atom handles. This list is unpacked, and then the function func
 * is applied to them. If the function returns an atom handle, then
 * this is returned. If the function does not return a handle, then
 * Handle::UNDEFINED is returned. If an error occurs during evaluation,
 * then a guile stack trace is logged in the OpenCog error log file,
 * and Handle::UNDEFINED is returned.
 */
Handle SchemeEval::apply(const std::string &func, Handle varargs)
{
	pexpr = &func;
	hargs = varargs;

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_lock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

	scm_with_guile(c_wrap_apply, this);

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_unlock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

	return hargs;
}

void * SchemeEval::c_wrap_apply(void * p)
{
	SchemeEval *self = (SchemeEval *) p;
	self->hargs = self->do_apply(*self->pexpr, self->hargs);
	return self;
}

/* ============================================================== */
/**
 * apply_generic -- apply named function func to arguments in ListLink
 * It is assumed that varargs is a ListLink, containing a list of
 * atom handles. This list is unpacked, and then the fuction func
 * is applied to them. The function is presumed to return some generic
 * scheme code, which is converted to a string and returned.
 *
 * XXX This seems awfully hacky to me -- is this really a good idea?
 * Isn't there some other, better way of accomplishing this?  My
 * gut instinct is the say "this should be reviewed and possibly
 * deprecated/removed". XXX
 */
std::string SchemeEval::apply_generic(const std::string &func, Handle varargs)
{
	pexpr = &func;
	hargs = varargs;

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_lock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

	scm_with_guile(c_wrap_apply_scm, this);

#ifdef WORK_AROUND_GUILE_THREADING_BUG
	thread_unlock();
#endif /* WORK_AROUND_GUILE_THREADING_BUG */

	return answer;
}

void * SchemeEval::c_wrap_apply_scm(void * p)
{
	logger().debug( "%s: calling wrap", __FUNCTION__);

	SchemeEval *self = (SchemeEval *) p;
	SCM genericAnswer = self->do_apply_scm(*self->pexpr, self->hargs);
	logger().debug( "%s: done", __FUNCTION__);
	self->answer = SchemeSmob::to_string(genericAnswer);
	logger().debug( "%s: answer: %s", __FUNCTION__, self->answer.c_str());

	return self;
}

#endif

/* ===================== END OF FILE ============================ */
