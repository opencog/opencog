/*
 * src/AtomSpace/StatisticsMonitor.cc
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

#include <stdlib.h>
#include <time.h>
#include "StatisticsMonitor.h"
#include "AtomTable.h"
#include "ClassServer.h"
#include "exceptions.h"
#include "HandleIterator.h"
#include "Link.h"
#include "Node.h"
#include "TLB.h"
#include "utils.h"

StatisticsMonitor* StatisticsMonitor::getInstance()
{
    static StatisticsMonitor* instance = new StatisticsMonitor();
    return instance;
}

StatisticsMonitor::StatisticsMonitor()
{
    init();
}

void StatisticsMonitor::init()
{
}

bool StatisticsMonitor::isCleared()
{
    return false;
}

int StatisticsMonitor::getLobeCycle()
{
    return 0;
}

void StatisticsMonitor::updateTypeCount(Type type, int delta)
{
}

void StatisticsMonitor::updateWeightSummation(Type type, float delta)
{
}

float StatisticsMonitor::getMeanWeight(Type type)
{
    return 0;
}

void StatisticsMonitor::updateHeatSummation(Type type, float delta)
{
}

float StatisticsMonitor::getMeanHeat()
{
    return 0;
}

float StatisticsMonitor::getMeanHeat(Type type)
{
    return 0;
}

int StatisticsMonitor::getAtomCount()
{
    return 0;
}

int StatisticsMonitor::getNodeCount()
{
    return 0;
}

int StatisticsMonitor::getLinkCount()
{
    return linkCount;
}

void StatisticsMonitor::atomChangeImportanceBin(Type type, int oldBin, int newBin)
{
}

void StatisticsMonitor::add(Atom* atom)
{
}


void StatisticsMonitor::remove(Atom* atom)
{
}

int StatisticsMonitor::getNodeImportanceBinCount(int i)
{
    return 0;
}

void StatisticsMonitor::reevaluateAllStatistics(const AtomTable& atomTable)
{
}

int StatisticsMonitor::getTypeCount(Type type)
{
    return 0;
}
