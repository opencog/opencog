/*
 * LojbanParser.h
 *
 * Copyright (C) 2020 SingularityNET Foundation
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

typedef void *HsStablePtr;  /* C representation of a Haskell StablePtr */

extern "C"
{
    opencog::Handle *lojban_parse(opencog::AtomSpace *, HsStablePtr, const char *);

    char* lojban_print(opencog::AtomSpace *, HsStablePtr, opencog::Handle *);

    HsStablePtr lojban_init();

    void lojban_exit(HsStablePtr);
}

