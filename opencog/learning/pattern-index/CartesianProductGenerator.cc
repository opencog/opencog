#include "CartesianProductGenerator.h"

using namespace opencog;

CartesianProductGenerator::CartesianProductGenerator(unsigned int n, unsigned int m, bool avoidEqual, bool triangular)
{
    if (triangular && !avoidEqual) {
        throw std::runtime_error("Invalid setup. \"triangular\" flag overides \"avoidEqual\"\n");
    }

    avoidEqualFlag = avoidEqual;
    triangularFlag = triangular;
    std::vector<unsigned int> v;
    for (unsigned int i = 0; i < n; i++) v.push_back(m);
    init(v);
}

CartesianProductGenerator::CartesianProductGenerator(const std::vector<unsigned int> &v, bool avoidEqual, bool triangular)
{
    avoidEqualFlag = avoidEqual;
    triangularFlag = triangular;
    init(v);
}

void CartesianProductGenerator::init(const std::vector<unsigned int> &v)
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
            if (triangularFlag) {
                cursorVector.push_back(i);
                if (i < (v.size() - 1)) {
                    base.push_back(i);
                } else {
                    base.push_back(v.at(i) - 1);
                }
            } else {
                cursorVector.push_back(0);
                base.push_back(v.at(i) - 1);
            }
        }
        if (avoidEqualFlag) {
            checkForRepetition();
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

void CartesianProductGenerator::drop(unsigned int pos)
{
    if (pos >= cursorVector.size()) {
        throw std::runtime_error("Invalid position\n");
    } 

    if (depletedFlag) {
        throw std::runtime_error("CartesianProductGenerator depleted\n");
    }

    for (unsigned int cursor = 0; cursor < pos; cursor++) {
        cursorVector[cursor] = base[cursor];
    }
}

void CartesianProductGenerator::checkForRepetition()
{
    if (! depletedFlag) {
        bool eqFlag = false;
        for (unsigned int i = 0; i < cursorVector.size(); i++) {
            for (unsigned int j = i + 1; j < cursorVector.size(); j++) {
                if (cursorVector.at(i) == cursorVector.at(j)) {
                    eqFlag = true;
                    break;
                }
            }
            if (eqFlag) {
                break;
            }
        }
        if (eqFlag) {
            generateNext();
        }
    }
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
            if (triangularFlag) {
                cursorVector[cursor] = cursor;
            } else {
                cursorVector[cursor] = 0;
            }
        }
        cursor++;
    }
    if (cursor == cursorVector.size()) {
        depletedFlag = true;
    } else {
        if (triangularFlag) {
            if (cursor > 0) {
                base[cursor -1] = cursorVector[cursor] - 1;
                for (int i = ((int) cursor) - 2; i >= 0; i--) {
                    base[i] = base[i + 1] - 1;
                }
            }
        }
        if (avoidEqualFlag) {
            checkForRepetition();
        }
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

void CartesianProductGenerator::printBaseForDebug(std::string prefix, std::string suffix) const
{
    printf("%s[", prefix.c_str());
    for (unsigned int i = 0; i < base.size(); i++) {
        printf("%u", base.at(i));
        if (i != (base.size() -1)) {
            printf(" ");
        }
    }
    printf("]%s", suffix.c_str());
}

