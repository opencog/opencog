/*
 * PythonShell.h
 *
 * Simple python shell
 *
 * @author Ramin Barati <rekino@gmail.com>
 * @date   2013-07-02
 *
 * @Note
 *   This code is almost identical to Linas' SchemeShell, so most of the credits
 *   goes to him.
 *
 * Reference:
 *   http://www.linuxjournal.com/article/3641?page=0,2
 *   http://www.codeproject.com/KB/cpp/embedpython_1.aspx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifdef HAVE_CYTHON

#include <opencog/cython/PythonEval.h>
#include <opencog/server/ConsoleSocket.h>
#include "PythonShell.h"

using namespace opencog;

PythonShell::PythonShell()
{
    normal_prompt = "py> ";
    pending_prompt = "... ";
    abort_prompt += normal_prompt;
    evaluator = NULL;
}

PythonShell::~PythonShell()
{
    // Don't delete, its currently set to a singleton instance.
    //	if (evaluator) delete evaluator;
}

/**
 * Register this shell with the console.
 */
void PythonShell::set_socket(ConsoleSocket *s)
{
    // Let the generic shell do the basic work.
    GenericShell::set_socket(s);

    // We are using a singlton instance here, because the current
    // Python evaluator isn't thread safe.  If/when it does become
    // thread-safe, we should probably go multi-threaed, instead.
    if (!evaluator) evaluator = &PythonEval::instance();
}

void PythonShell::eval(const std::string &expr, ConsoleSocket *s)
{
    bool selfie = self_destruct;
    self_destruct = false;
    GenericShell::eval(expr, s);
    if (selfie) {
        self_destruct = true;

        // Eval an empty string as a end-of-file marker. This is needed
        // to flush pending input in the python sehll, as otherwise,
        // there is no way to know that no more python input will
        // arrive!
        GenericShell::do_eval("");
    }
}

void PythonShell::socketClosed(void)
{
    // Eval an empty string as a end-of-file marker. This is needed
    // to flush pending input in the python sehll, as otherwise,
    // there is no way to know that no more python input will
    // arrive!
    GenericShell::do_eval("");
    GenericShell::socketClosed();
}

#endif
