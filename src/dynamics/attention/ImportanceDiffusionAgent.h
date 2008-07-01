/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#ifndef _IMPORTANCE_DIFFUSION_AGENT_H
#define _IMPORTANCE_DIFFUSION_AGENT_H

#include <string>
#include <math.h>

#include <Logger.h>

#include <AtomSpace.h>
#include <MindAgent.h>
#include <AttentionValue.h>

namespace opencog
{

class CogServer;

/** Spreads short term importance along HebbianLinks using a diffusion approach.
 *
 * Currently spreads along Symmetric and Inverse HebbianLinks.
 *
 * @todo Spread along asymmetric hebbian links too.
 * @todo Optionally spread long term importance.
 */
class ImportanceDiffusionAgent : public MindAgent
{

private:
    AtomSpace* a;

    //! Total amount spread during recent runs.
    opencog::recent_val<long> amountSpread;

    //! Spread importance along Hebbian links.
    void spreadImportance();

public:

    ImportanceDiffusionAgent();
    virtual ~ImportanceDiffusionAgent();
    virtual void run(CogServer *server);

}; // class

}; // namespace
#endif // _IMPORTANCE_DIFFUSION_AGENT_H
