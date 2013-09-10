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

#ifndef PYTHONSHELL_H
#define PYTHONSHELL_H

#include <string>

#include <opencog/cython/PythonEval.h>
#include <opencog/server/ConsoleSocket.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class PythonShellModule;

class PythonShell: public GenericShell
{
    friend class PythonShellModule;
private:
    PythonEval *evaluator;

    std::string normal_prompt;
    std::string pending_prompt;
    std::string abort_prompt;
    const std::string& get_prompt(void);
    bool show_output;
    bool show_prompt;

    ConsoleSocket *socket;
    std::string do_eval(const std::string &);
    bool self_destruct;

protected:
    void set_socket(ConsoleSocket *);

public:
    PythonShell(void);
    virtual ~PythonShell();

    virtual void eval(const std::string &, ConsoleSocket *);
    virtual void socketClosed(void);

    void hush_output(bool);
    void hush_prompt(bool);
};

/** @}*/
}

#endif // PYTHONSHELL_H
#endif // HAVE_CYTHON
