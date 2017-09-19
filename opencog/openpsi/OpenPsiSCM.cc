/*
 * OpenPsiSCM.cc
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

#include <opencog/guile/SchemePrimitive.h>

#include "OpenPsiImplicator.h"
#include "OpenPsiSCM.h"

using namespace opencog;

/**
 * The constructor for OpenPsiSCM.
 */
OpenPsiSCM::OpenPsiSCM()
{
  static bool is_init = false;
  if (is_init) return;
  is_init = true;
  scm_with_guile(init_in_guile, this);
}

/**
 * The main init function for the OpenPsiSCM object.
 */
void OpenPsiSCM::init()
{
  define_scheme_primitive("psi-satisfy", &OpenPsiSCM::satisfiable,
    this, "openpsi");
}

TruthValuePtr OpenPsiSCM::satisfiable(const Handle& himplication)
{
  // TODO: Rename to psi-satisfiable? once c++ cache is implemented.
  AtomSpace *as = SchemeSmob::ss_get_env_as("psi-satisfy");
  OpenPsiImplicator imply(as);
  return imply.check_satisfiability(himplication);
}

/**
 * Init function for using with scm_with_guile.
 *
 * Creates the openpsi scheme module and uses it by default.
 *
 * @param self  pointer to the OpenPsiSCM object
 */
void* OpenPsiSCM::init_in_guile(void* self)
{
  scm_c_define_module("opencog openpsi", init_in_module, self);
  scm_c_use_module("opencog openpsi");
  return NULL;
}

/**
 * The main function for defining stuff in the openpsi scheme module.
 *
 * @param data  pointer to the OpenPsiSCM object
 */
void OpenPsiSCM::init_in_module(void* data)
{
  OpenPsiSCM* self = (OpenPsiSCM*) data;
  self->init();
}

void opencog_openpsi_init(void)
{
  static OpenPsiSCM openpsi;
}
