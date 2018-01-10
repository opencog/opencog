/* There is no include guard here - by purpose. This file can be included
 * after system includes that redefine the assert() macro.
 * The actual problem for which this file got separated from utilities.h
 * happens in the sat-solver code, when local include files include
 * Solver.h which in turn includes the system's assert.h. */

#include "error.h" /* for prt_error() */

#ifndef STRINGIFY
#define STR(x) #x
#define STRINGIFY(x) STR(x)
#endif /* STRINGIFY */

#define FILELINE __FILE__ ":" STRINGIFY(__LINE__)

#ifdef _WIN32
	#define DEBUG_TRAP (*((volatile int*) 0x0) = 42)
#elif defined GNUC
	#define DEBUG_TRAP __builtin_trap()
#else
	#define DEBUG_TRAP ((void(*)())0)()
#endif

/* FIXME:
 * 1. If the error_handler is not NULL, use prt_error() too (after calling
 *    fprintf()), in order to allow an error_handler to log, produce trace, or
 *    show the assert() message in a (possibly pop-up) window.
 * 2. Don't use DEBUG_TRAP (or exit) directly, but instead call a function
 *    pointer like lg_exit(code) to allow the LG library to be embedded in an
 *    application like an editor. If not set, the default will still be
 *    DEBUG_TRAP. */
#define assert(ex, ...) {                                                   \
	if (!(ex)) {                                                             \
		fprintf(stderr, "Fatal error: \nAssertion (" #ex ") failed at " FILELINE ": " __VA_ARGS__);  \
		fprintf(stderr, "\n");                                                \
		DEBUG_TRAP;  /* leave stack trace in debugger */                      \
	}                                                                        \
}
