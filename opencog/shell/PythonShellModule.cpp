#ifdef HAVE_CYTHON

#include "PythonShellModule.h"

namespace opencog
{

PythonShellModule::PythonShellModule()
{
}

PythonShellModule::~PythonShellModule()
{
    shellout_unregister();
    do_eval_unregister();
}

void PythonShellModule::init(void)
{
    shellout_register();
    do_eval_register();
}

std::string PythonShellModule::shellout(Request *req, std::list<std::string> args)
{
    ConsoleSocket *s = dynamic_cast<ConsoleSocket*>(req->getRequestResult());
    if (!s)
        throw RuntimeException(TRACE_INFO, "Invalid RequestResult object"
               " for PythonShellModule: a ConsoleSocket object was expected.");

    // TODO create PythonShell class ----------------------------------

    //SchemeShell *sh = new SchemeShell();
    //sh->set_socket(s);

    bool hush = false;
    if (0 < args.size())
    {
        std::string &arg = args.front();
        if (arg.compare("quiet") || arg.compare("hush")) hush = true;
    }
    //sh->hush_prompt(hush);

    if (hush) return "";

    return "Entering python shell; use ^D or a single . on a "
           "line by itself to exit. (not working right now)";
}

std::string PythonShellModule::do_eval(Request *req, std::list<std::string> args)
{
    // Needs to join the args back up into one string.
    std::string expr;
    std::string out = "test";

    // Adds an extra space on the end, but that doesn't matter.
    foreach(std::string arg, args)
    {
        expr += arg + " ";
    }

    PythonEval& eval = PythonEval::instance();
//	out = eval.eval(expr);
    // May not be necessary since an error message and backtrace are provided.
//	if (eval.eval_error()) {
//		out += "An error occurred\n";
//	}
//	if (eval.input_pending()) {
//		out += "Invalid Scheme expression: missing something";
//	}
//	eval.clear_pending();

    return out;
}

}
 #endif
