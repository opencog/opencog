/**
 * ensemble_params.h ---
 *
 * Copyright (C) 2014 Aidyia Limited
 *
 * Authors: Linas Vepstas
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

#ifndef _OPENCOG_ENSEMBLE_PARAMS_H
#define _OPENCOG_ENSEMBLE_PARAMS_H

namespace opencog {
namespace moses {

struct ensemble_parameters
{
	ensemble_parameters() :
		experts(false),
		num_to_promote(1)
	{}

	bool experts;        // do "ensemble-of-experts" boosting.
	int num_to_promote;  // max number of demes to accept into ensemble,
	                     // per learning iteration.
};

}}; // namespace opencog::moses

#endif // _OPENCOG_ENSEMBLE_PARAMS_H


