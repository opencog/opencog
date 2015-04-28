/*
 * opencog/embodiment/Learning/behavior/BehaviorCategory.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#include "BehaviorCategory.h"

using namespace behavior;

BehaviorCategory::~BehaviorCategory()
{
}

BehaviorCategory::BehaviorCategory(AtomSpace* a) : atomspace(a)
{
}

void BehaviorCategory::addCompositeBehaviorDescription(const CompositeBehaviorDescription &bd)
{
    entries.push_back(bd);
}

std::string BehaviorCategory::toString()
{

    std::string answer = "";

    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append(entries[i].toString());
        if (i != (entries.size() - 1)) {
            answer.append("\n");
        }
    }

    return answer;
}

std::string BehaviorCategory::toStringHandles()
{

    std::string answer = "";

    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append(entries[i].toStringHandles());
        if (i != (entries.size() - 1)) {
            answer.append("\n");
        }
    }

    return answer;
}

std::string BehaviorCategory::toStringTimeline()
{

    std::string answer = "";

    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append(entries[i].toStringTimeline());
        if (i != (entries.size() - 1)) {
            answer.append("\n");
        }
    }

    return answer;
}

int BehaviorCategory::getSize() const
{
    return entries.size();
}

const std::vector<CompositeBehaviorDescription> &BehaviorCategory::getEntries() const
{
    return entries;
}

bool BehaviorCategory::empty()
{
    return entries.empty();
}

void BehaviorCategory::clear()
{
    entries.clear();
}

// ********************************************************************************
// Private API

