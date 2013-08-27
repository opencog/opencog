/*
 * opencog/util/ansi.h
 *
 * Copyright (C) 2010 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@opencog.org>
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
#include <string>

#include <opencog/util/Config.h>
namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

//!@{
//! color codes in ANSI
static const char* const COLOR_OFF = "\033[0m";
static const char* const BRIGHT = "\033[1m";
static const char* const RED = "\033[31m";
static const char* const GREEN = "\033[32m";
static const char* const YELLOW = "\033[33m";
static const char* const BLUE = "\033[34m";
static const char* const MAGENTA = "\033[35m";
static const char* const CYAN = "\033[36m";
static const char* const WHITE = "\033[37m";
//!@}

//! inserts the code only if \b ANSI_ENABLED variable is set
inline void ansi_code(std::string &s,const std::string &code) {
    if (config().get_bool("ANSI_ENABLED")) s.append(code);
}

//!@{
//! appends the code if \b ANSI_ENABLED variable is set
inline void ansi_off(std::string &s) { ansi_code(s,COLOR_OFF); }
inline void ansi_bright(std::string &s) { ansi_code(s,BRIGHT); }
inline void ansi_red(std::string &s) { ansi_code(s,RED); }
inline void ansi_green(std::string &s) { ansi_code(s,GREEN); }
inline void ansi_yellow(std::string &s) { ansi_code(s,YELLOW); }
inline void ansi_blue(std::string &s) { ansi_code(s,BLUE); }
inline void ansi_magenta(std::string &s) { ansi_code(s,MAGENTA); }
inline void ansi_cyan(std::string &s) { ansi_code(s,CYAN); }
inline void ansi_white(std::string &s) { ansi_code(s,WHITE); }
//!@}

/** @}*/
}
