/*
 * SchemeShell.c
 *
 * Simple scheme shell
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifdef HAVE_GUILE

#include "SchemeShell.h"

#include <libguile.h>
#include <libguile/backtrace.h>
#include <libguile/lang.h>

#include "platform.h"
#include "SchemeSmob.h"

using namespace opencog;

bool SchemeShell::is_inited = false;

SchemeShell::SchemeShell(void)
{
	if (!is_inited)
	{
		is_inited = true;
		scm_init_guile();
		scm_init_debug();
		scm_init_backtrace();
		scm_init_strports(); // is this really needed ?
	}

	funcs = new SchemeSmob();
	pending_input = false;
	show_output = true;
	input_line = "";
	normal_prompt = "guile> ";
	pending_prompt = "... ";

	outport = scm_open_output_string();
	scm_set_current_output_port(outport);
}

void SchemeShell::hush_output(bool hush)
{
	show_output = !hush;
}

/* ============================================================== */

std::string SchemeShell::prt(SCM node)
{
	if (scm_is_pair(node))
	{
		std::string str = "(";
      SCM node_list = node;
		const char * sp = "";
      do
      {
			str += sp;
			sp = " ";
         node = SCM_CAR (node_list);
         str += prt (node);
         node_list = SCM_CDR (node_list);
      }
      while (scm_is_pair(node_list));

		// Print the rest -- the CDR part
		if (!scm_is_null(node_list)) 
		{
			str += " . ";
			str += prt (node_list);
		}
		str += ")";
		return str;
   }
	else if (scm_is_true(scm_symbol_p(node))) 
	{
		node = scm_symbol_to_string(node);
		char * str = scm_to_locale_string(node);
		// std::string rv = "'";  // print the symbol escape
		std::string rv = "";      // err .. don't print it
		rv += str;
		free(str);
		return rv;
	}
	else if (scm_is_true(scm_string_p(node))) 
	{
		char * str = scm_to_locale_string(node);
		std::string rv = "\"";
		rv += str;
		rv += "\"";
		free(str);
		return rv;
	}

	else if (SCM_SMOB_PREDICATE(SchemeSmob::cog_handle_tag, node))
	{
		return SchemeSmob::handle_to_string(node);
	}

	else if (SCM_SMOB_PREDICATE(SchemeSmob::cog_misc_tag, node))
	{
		return SchemeSmob::misc_to_string(node);
	}

	else if (scm_is_number(node)) 
	{
		#define NUMBUFSZ 60
		char buff[NUMBUFSZ];
		if (scm_is_signed_integer(node, INT_MIN, INT_MAX))
		{
			snprintf (buff, NUMBUFSZ, "%ld", (long) scm_to_long(node));
		}
		else if (scm_is_unsigned_integer(node, 0, UINT_MAX))
		{
			snprintf (buff, NUMBUFSZ, "%lu", (unsigned long) scm_to_ulong(node));
		}
		else if (scm_is_real(node))
		{
			snprintf (buff, NUMBUFSZ, "%g", scm_to_double(node));
		}
		else if (scm_is_complex(node))
		{
			snprintf (buff, NUMBUFSZ, "%g +i %g", 
				scm_c_real_part(node),
				scm_c_imag_part(node));
		}
		else if (scm_is_rational(node))
		{
			std::string rv;
			rv = prt(scm_numerator(node));
			rv += "/";
			rv += prt(scm_denominator(node));
			return rv;
		}
		return buff;
	}
	else if (scm_is_true(scm_char_p(node))) 
	{
		std::string rv;
		rv = (char) scm_to_char(node);
		return rv;
	}
	else if (scm_is_true(scm_boolean_p(node))) 
	{
		if (scm_to_bool(node)) return "#t";
		return "#f";
	}
	else if (SCM_NULL_OR_NIL_P(node)) 
	{
		// scm_is_null(x) is true when x is SCM_EOL
		// SCM_NILP(x) is true when x is SCM_ELISP_NIL
		return "()";
	}
	else if (scm_is_eq(node, SCM_UNDEFINED))
	{
		return "undefined";
	}
	else if (scm_is_eq(node, SCM_EOF_VAL))
	{
		return "eof";
	}
	else if (scm_is_eq(node, SCM_UNSPECIFIED))
	{
		return "";
	}
	else if (scm_is_true(scm_dynamic_object_p(node))) 
	{
		return "#<dynamic object (XXX add name here)>";
	}
	else if (scm_is_true(scm_port_p(node))) 
	{
		return "#<port (XXX add name here)>";
	}
	else if (scm_is_true(scm_procedure_p(node))) 
	{
		return "#<port (XXX add name here)>";
	}
	else if (scm_subr_p(node)) 
	{
		return "#<subr (XXX add name here)>";
	}
	else if (scm_is_true(scm_operator_p(node))) 
	{
		return "#<operator (XXX add name here)>";
	}
	else if (scm_is_true(scm_entity_p(node))) 
	{
		return "#<entity (XXX add name here)>";
	}
	else if (scm_is_true(scm_variable_p(node))) 
	{
		return "#<variable (XXX add name here)>";
	}
	else
	{
		fprintf (stderr, "Error: unhandled type for guile printing: %p\n",
			node);
		return "#opencog-guile-error: unknown type";
	}

	return "";
}

/* ============================================================== */

SCM SchemeShell::catch_handler_wrapper (void *data, SCM tag, SCM throw_args)
{
	SchemeShell *ss = (SchemeShell *)data;
	return ss->catch_handler(tag, throw_args);
}

SCM SchemeShell::catch_handler (SCM tag, SCM throw_args)
{
	// Check for read error. If a read error, then wait for user to correct it.
	SCM re = scm_symbol_to_string(tag);
	char * restr = scm_to_locale_string(re);
	pending_input = false;
	if (0 == strcmp(restr, "read-error"))
	{
		pending_input = true;
		free(restr);

		// quit accumulating text on escape (^[), cancel (^X) or ^C
		char c = input_line[input_line.length()-1];
		if ((0x6 == c) || (0x16 == c) || (0x18 == c) || (0x1b == c))
		{
			pending_input = false;
			input_line = "";
		}
		return SCM_EOL;
	}
	free(restr);

	// If its not a read error, then its a regular error; report it.
	caught_error = true;

	/* get string port into which we write the error message and stack. */
	error_string_port = scm_open_output_string();
	SCM port = error_string_port;

	if (scm_is_true(scm_list_p(throw_args)) && (scm_ilength(throw_args) >= 4))
	{
		SCM stack   = scm_make_stack (SCM_BOOL_T, SCM_EOL);
		SCM subr    = SCM_CAR (throw_args);
		SCM message = SCM_CADR (throw_args);
		SCM parts   = SCM_CADDR (throw_args);
		SCM rest    = SCM_CADDDR (throw_args);
		if (scm_is_true (stack))
		{
			SCM highlights;

			if (scm_is_eq (tag, scm_arg_type_key) ||
			    scm_is_eq (tag, scm_out_of_range_key))
				highlights = rest;
			else
				highlights = SCM_EOL;

			scm_puts ("Backtrace:\n", port);
			scm_display_backtrace_with_highlights (stack, port,
			      SCM_BOOL_F, SCM_BOOL_F, highlights);
			scm_newline (port);
		}
		scm_i_display_error (stack, port, subr, message, parts, rest);
	}
	else
	{
		printf("ERROR: thow args are unexpectedly short!\n");
	}

#if 0
	/* find the stack, and conditionally display it */
	SCM the_stack = scm_fluid_ref(SCM_CDR(scm_the_last_stack_fluid_var));
	if (the_stack != SCM_BOOL_F)
	{
		scm_display_backtrace(the_stack, port, SCM_UNDEFINED, SCM_UNDEFINED);
	}
#endif

	return SCM_BOOL_F;
}

/* ============================================================== */

/**
 * Evaluate the expression
 */
std::string SchemeShell::eval(const std::string &expr)
{
	input_line += expr;

	caught_error = false;
	pending_input = false;
	SCM rc = scm_internal_catch (SCM_BOOL_T,
	            (scm_t_catch_body) scm_c_eval_string, (void *) input_line.c_str(),
	            SchemeShell::catch_handler_wrapper, this);

	if (pending_input)
	{
		if (show_output)
			return pending_prompt;
		else
			return "";
	}
	pending_input = false;
	input_line = "";

	if (caught_error)
	{
		std::string rv;
		rc = scm_get_output_string(error_string_port);
		char * str = scm_to_locale_string(rc);
		rv = str;
		free(str);
		scm_close_port(error_string_port);

		scm_truncate_file(outport, scm_from_uint16(0));

		rv += "\n";
		rv += normal_prompt;
		return rv;
	}
	else
	{
		if (show_output)
		{
			std::string rv;
			// First, we get the contents of the output port,
			// and pass that on.
			SCM out = scm_get_output_string(outport);
			char * str = scm_to_locale_string(out);
			rv = str;
			free(str);
			scm_truncate_file(outport, scm_from_uint16(0));

			// Next, we append the "interpreter" output
			rv += prt(rc);
			rv += "\n";

			rv += normal_prompt;
			return rv;
		}
		else
			return "";
	}
	return "#<Error: Unreachable statement reached>";
}

#endif
/* ===================== END OF FILE ============================ */
