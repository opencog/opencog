/**
 * BehaviorCategory.cc
 *
 * $Header$
 *
 * Author: Andre Senna
 * Creation: Wed Aug 22 12:57:33 BRT 2007
 */

#include "BehaviorCategory.h"

using namespace behavior;

BehaviorCategory::~BehaviorCategory() {
}

BehaviorCategory::BehaviorCategory() {
}

void BehaviorCategory::addCompositeBehaviorDescription(const CompositeBehaviorDescription &bd) {
    entries.push_back(bd);
}

std::string BehaviorCategory::toString() {

    std::string answer = "";

    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append(entries[i].toString());
        if (i != (entries.size() - 1)) {
            answer.append("\n");
        }
    }

    return answer;
}

std::string BehaviorCategory::toStringHandles() {

    std::string answer = "";

    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append(entries[i].toStringHandles());
        if (i != (entries.size() - 1)) {
            answer.append("\n");
        }
    }

    return answer;
}

std::string BehaviorCategory::toStringTimeline() {

    std::string answer = "";

    for (unsigned int i = 0; i < entries.size(); i++) {
        answer.append(entries[i].toStringTimeline());
        if (i != (entries.size() - 1)) {
            answer.append("\n");
        }
    }

    return answer;
}

int BehaviorCategory::getSize() const {
    return entries.size();
}   

const std::vector<CompositeBehaviorDescription> &BehaviorCategory::getEntries() const {
    return entries;
}

bool BehaviorCategory::empty() {
  return entries.empty();
}

void BehaviorCategory::clear() {
  entries.clear();
}

// ********************************************************************************
// Private API

