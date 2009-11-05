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
		static void * c_wrap_eval_h(void *);
		const std::string *pexpr;
		std::string answer;

		std::string input_line;
		bool pending_input;

		// straight-up evaluation
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

		// Make constructor, destructor private; force
		// everyone to use the singleton instance, for now.
		SchemeEval(void);
		~SchemeEval();
		static SchemeEval* singletonInstance;

	public:
                
                /**
                 * This abstract class can be extended to create custom
                 * commands that can be called inside a Scheme script.
                 * i.e.
                 * one can define a class like
                 * class Foo : public SchemeEval::SchemeFunction {
                 *   public:
                 *      Foo( void ) : SchemeEval::SchemeFunction::SchemeFunction( "foo", 2, 0, 0 ) { }
                 *
                 *      virtual SchemeEval::SchemeFunction::FunctionPointer getFunctionPointer( void ) {
                 *           return (SchemeEval::SchemeFunction::FunctionPointer)&FOO::execute;
                 *      }
                 *   private:
                 *      static SCM execute( SCM arg1, SCM arg2 ) {
                 *           // foo code goes here
                 *      }
                 * };
                 * 
                 * then register the new function calling:
                 * Foo *foo = new Foo();
                 * SchemeEval::instance( ).register_function( foo );
                 * ...
                 * 
                 */
                class SchemeFunction {
                public:
                    typedef SCM (*FunctionPointer)( );

                    SchemeFunction(const std::string& name, int requiredArgs, int optionalArgs, int restArgs ) :
                    name( name ), requiredArgs( requiredArgs ), optionalArgs( optionalArgs ), 
                        restArgs( restArgs ) { }
                    
                    inline const std::string& getName( void ) const { return this->name; }
                    inline int getNumberOfRequiredArguments( void ) const { return this->requiredArgs; }
                    inline int getNumberOfOptionalArguments( void ) const { return this->optionalArgs; }
                    inline int getNumberOfRestArguments( void ) const { return this->restArgs; }

                    virtual FunctionPointer getFunctionPointer( void ) = 0;
                    
                protected:
                    std::string name;
                    int requiredArgs, optionalArgs, restArgs;
                };

                bool register_function(SchemeFunction*);
                
		std::string eval(const std::string &);
		Handle eval_h(const std::string &);
		Handle apply(const std::string& func, Handle varargs);
		std::string apply_generic(const std::string& func, Handle varargs);

		bool input_pending(void);
		void clear_pending(void);
		bool eval_error(void);

		// Someone thinks that there some scheme threading bug somewhere,
		// and the current hack around this is to use a singleton instance.
		static SchemeEval& instance(void)
		{
			if (!singletonInstance) 
				singletonInstance = new SchemeEval();
			return *singletonInstance;
		}
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
