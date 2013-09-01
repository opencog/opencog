/*
 * PythonShellModule.h
 *
 * Module for starting up python shell
 *
 * @author Ramin Barati <rekino@gmail.com>
 * @date   2013-07-02
 *
 * @Note
 *   This code is almost identical to Linas' SchemeShellModule, so most of the credits
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

#ifndef PYTHONSHELLMODULE_H
#define PYTHONSHELLMODULE_H

#include <string>

#include <opencog/shell/PythonShell.h>
#include <opencog/server/Request.h>
#include <opencog/server/CogServer.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class PythonShellModule : public Module
{
private:
    DECLARE_CMD_REQUEST(PythonShellModule, "py", shellout,
        "Enter the python shell",
        "Usage: py [hush|quiet]\n\n"
        "Enter the python interpreter shell. This shell provides a rich\n"
        "and easy-to-use environment for creating, deleting and manipulating\n"
        "OpenCog atoms and truth values.\n\n"
        "If 'hush' or 'quiet' is specified after the command, then the prompt\n"
        "will not be returned.  This is nice when catting large scripts using\n"
        "netcat, as it avoids printing garbage when the scripts work well.\n",
        true, false)

    DECLARE_CMD_REQUEST(PythonShellModule, "py-eval", do_eval,
        "Run some python code",
        "Usage: py-eval <python code>\n\n"
        "Evaluate the specified Python code. It does not need to be quoted.",
        false, false)

public:
    PythonShellModule(CogServer&);
    ~PythonShellModule();

    static const char *id(void);
    virtual void init(void);
};

/** @}*/
}

#endif // PYTHONSHELLMODULE_H

#endif // HAVE_CYTHON
