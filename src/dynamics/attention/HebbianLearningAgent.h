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

/**
 * HebbianLearningAgent.h
 *
 * Simple Hebbian Learning, currently only updates existing Hebbian Links.
 */

#ifndef _HEBBIAN_LEARNING_AGENT_H
#define _HEBBIAN_LEARNING_AGENT_H

#include <string>
#include <Logger.h>

#include <AtomSpace.h>
#include <MindAgent.h>
#include <AttentionValue.h>

namespace opencog {

class CogServer;

class HebbianLearningAgent : public MindAgent
{

    private:
	AtomSpace* a;

	float targetConjunction(std::vector<Handle> handles);
	float getNormSTI(AttentionValue::sti_t s);
	std::vector<Handle>& moveSourceToFront(std::vector<Handle> &outgoing);
    public:
	// Convert links to/from inverse as necessary.
	bool convertLinks;
	// Maximum LTI of a link that can be converted.
	AttentionValue::lti_t conversionThreshold;

	HebbianLearningAgent();
	virtual ~HebbianLearningAgent();
	virtual void run(CogServer *server);

	void hebbianLearningUpdate();

}; // class

}; // namespace
#endif // _HEBBIAN_LEARNING_AGENT_H

