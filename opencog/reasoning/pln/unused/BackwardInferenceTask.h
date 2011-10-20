/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
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

#ifndef _BACKWARDINFERENCETASK_H_
#define _BACKWARDINFERENCETASK_H_

namespace opencog { namespace pln
{
	
class InferenceTaskParameters;

/**
	The goal/BackwardInferenceAgent should be replaced by this kind of a structure,
	which is not yet implemented beyond this simple skeleton.
*/

//! @todo this should inherit from Task... but Task doesn't exist in OpenCog yet
class BackwardInferenceTask
{
	const InferenceTaskParameters& pars;
	bool finished;
public:
	BackwardInferenceTask(const InferenceTaskParameters& _pars);

	virtual ~BackwardInferenceTask();
	
	virtual void cycle();

	/**
	 * Task Interface: This method is called when the Task has spent the amount of 
	 * resources it was granted by its EffortMonitor.
	 */ 	
	virtual void finish();
	bool ok();
};

}}

#endif //_BACKWARDINFERENCETASK_H_
