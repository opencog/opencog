#include "CartesianProductGenerator.h"

using namespace opencog;

CartesianProductGenerator::CartesianProductGenerator(const std::vector<unsigned int> &v)
{
    if (v.size() == 0) {
        depletedFlag = true;
    } else {
        depletedFlag = false;
        for (unsigned int i = 0; i < v.size(); i++) {
            if (v.at(i) == 0) {
                depletedFlag = true;
                break;
            }
            cursorVector.push_back(0);
            base.push_back(v.at(i) - 1);
        }
    }
}

CartesianProductGenerator::~CartesianProductGenerator() 
{
}

bool CartesianProductGenerator::depleted() const
{
    return depletedFlag;
}

unsigned int CartesianProductGenerator::at(unsigned int pos) const
{
    if (pos >= cursorVector.size()) {
        throw std::runtime_error("Invalid position\n");
    } 

    if (depletedFlag) {
        throw std::runtime_error("CartesianProductGenerator depleted\n");
    }

    return cursorVector.at(pos);
}

void CartesianProductGenerator::generateNext()
{
    if (depletedFlag) {
        throw std::runtime_error("CartesianProductGenerator depleted\n");
    }

    unsigned int cursor = 0;
    while (cursor < cursorVector.size()) {
        if (cursorVector.at(cursor) < base.at(cursor)) {
            cursorVector[cursor]++;
            break;
        } else {
            cursorVector[cursor] = 0;
        }
        cursor++;
    }

    if (cursor == cursorVector.size()) {
        depletedFlag = true;
    }
}

void CartesianProductGenerator::printForDebug(std::string prefix, std::string suffix) const
{
    printf("%s(", prefix.c_str());
    for (unsigned int i = 0; i < cursorVector.size(); i++) {
        printf("%u", cursorVector.at(i));
        if (i != (cursorVector.size() -1)) {
            printf(" ");
        }
    }
    printf(")%s", suffix.c_str());
}

