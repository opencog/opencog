/*
 * OpenPsiSCM.h
 *
 * Copyright (C) 2017 MindCloud
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

#ifndef OPENPSI_H
#define OPENPSI_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/truthvalue/TruthValue.h>

namespace opencog
{

class OpenPsiSCM
{
  public:
    OpenPsiSCM();

  private:
    static void* init_in_guile(void*);
    static void init_in_module(void*);
    void init(void);

    TruthValuePtr satisfiable(const Handle& himplication);

};

} // namespace opencog

extern "C" {
void opencog_openpsi_init(void);
};

#endif  // OPENPSI_H
