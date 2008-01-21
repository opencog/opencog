/**
 * StatisticsMonitor.cc
 *
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
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

StatisticsMonitor* StatisticsMonitor::getInstance() {
	static StatisticsMonitor* instance = new StatisticsMonitor();
    return instance;
}

StatisticsMonitor::StatisticsMonitor() {
    init();
}

void StatisticsMonitor::init() {
}

bool StatisticsMonitor::isCleared() {
    return false;
}

int StatisticsMonitor::getLobeCycle() {
    return 0;
}

void StatisticsMonitor::updateTypeCount(Type type, int delta) {
}

void StatisticsMonitor::updateWeightSummation(Type type, float delta) {
}

float StatisticsMonitor::getMeanWeight(Type type) {
    return 0;
}

void StatisticsMonitor::updateHeatSummation(Type type, float delta) {
}

float StatisticsMonitor::getMeanHeat() {
    return 0;
}

float StatisticsMonitor::getMeanHeat(Type type) {
    return 0;
}

int StatisticsMonitor::getAtomCount() {
    return 0;
}

int StatisticsMonitor::getNodeCount() {
    return 0;
}

int StatisticsMonitor::getLinkCount() {
    return linkCount;
}

void StatisticsMonitor::atomChangeImportanceBin(Type type, int oldBin, int newBin) {
}

void StatisticsMonitor::add(Atom* atom) {
}


void StatisticsMonitor::remove(Atom* atom) {
}

int StatisticsMonitor::getNodeImportanceBinCount(int i) {
    return 0;
}

void StatisticsMonitor::reevaluateAllStatistics(const AtomTable& atomTable) {
}

int StatisticsMonitor::getTypeCount(Type type) {
    return 0;
}
