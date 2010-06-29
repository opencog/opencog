/*
 * opencog/util/oc_assert.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#include "oc_assert.h"
#include "exceptions.h"

void opencog::cassert(const char * trace, bool condition, const char *fmt, ...)
{
    if (condition) return;

    va_list        ap;
    va_start(ap, fmt);

    char * c_msg = new char[strlen(fmt) + strlen(trace) + 1];
    *c_msg = '\0'; // empty c-string

    strcat(c_msg, fmt);
    strcat(c_msg, trace);

    AssertionException ex = AssertionException(c_msg, ap);
    va_end(ap);

    delete [] c_msg;
    throw ex;
}

void opencog::cassert(const char* trace, bool condition, const std::string& msg) {
    opencog::cassert(trace, condition, msg.c_str());
}

void opencog::cassert(const char * trace, bool condition)
{

    if (condition) return;

    AssertionException ex = AssertionException(trace);
    throw ex;
}

