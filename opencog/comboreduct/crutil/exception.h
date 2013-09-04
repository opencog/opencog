/*
 * opencog/comboreduct/crutil/exception.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _COMBOREDUCT_EXCEPTION_H
#define _COMBOREDUCT_EXCEPTION_H

#include <string>
#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace combo {

/// Base class for all combo user-error exceptions
///
/// Non-user errors, such as internal bugs, use ComboException
/// (defined in util/exceptions.h) so that a stack trace gets printed
/// to the log-file.
class ComboReductException
{
protected:
    std::string _message;
public:
    ComboReductException();
    ComboReductException(std::string m = "");

    std::string get_message() const;
};

/// Overflow/ divide-by-zero exception during evaluation.
// XXX should change this to "OverflowException"
class EvalException : public ComboReductException
{
    vertex _vertex;
public:
    EvalException();
    EvalException(vertex v = vertex());

    vertex get_vertex() const;
};

/// Typing problem
class TypeCheckException : public ComboReductException
{
    int _arg;
public:
    TypeCheckException();
    TypeCheckException(int arg = 0);
};

}} // ~namespaces combo opencog

#endif
