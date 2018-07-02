#include "CombinationGenerator.h"

using namespace opencog;

CombinationGenerator::CombinationGenerator(unsigned int n, bool avoidAllZero, bool avoidOnes)
{
    if (n == 0) {
        depletedFlag = true;
    } else {
        for (unsigned int i = 0; i < n; i++) {
            elements.push_back(false);
        }
        depletedFlag = false;
        combinationAlgorithm = ALL_COMB;
        avoidAllOne = avoidOnes;
        countOnes = 0;
        if (avoidAllZero) {
            generateNext();
        }
    }
}

CombinationGenerator::CombinationGenerator(unsigned int n, unsigned int k)
{
    if ((n == 0) || (k == 0) || (k > n)) {
        throw std::runtime_error("Error creating CombinationGenerator\n");
    } 

    for (unsigned int i = 0; i < n; i++) {
        elements.push_back(i < k);
    }
    depletedFlag = false;
    countOnes = k;
    combinationAlgorithm = K_COMB;
    avoidAllZero = false;
    avoidAllOne = false;
}

CombinationGenerator::~CombinationGenerator() 
{
}

bool CombinationGenerator::at(unsigned int pos) const
{
    if (pos >= elements.size()) {
        throw std::runtime_error("Invalid element\n");
    } 

    if (depletedFlag) {
        throw std::runtime_error("CombinationGenerator depleted\n");
    }

    return elements.at(pos);
}

bool CombinationGenerator::depleted() const
{
    return depletedFlag;
}

void CombinationGenerator::generateNext()
{
    if (depletedFlag) {
        throw std::runtime_error("CombinationGenerator depleted\n");
    }

    switch (combinationAlgorithm) {
        case ALL_COMB: {
            CombinationGenerator::generateNextAllComb();
            break;
        } 
        case K_COMB: {
            CombinationGenerator::generateNextKComb();
            break;
        } 
        default: throw std::runtime_error("Unknown combination algorithm\n");
    }
}

void CombinationGenerator::generateNextKComb()
{
    unsigned int cursor = 0;
    unsigned int n = elements.size();
    int selected = -1;
    while (cursor < (n - 1)) {
        if (elements.at(cursor) && (! elements.at(cursor + 1))) {
            selected = cursor;
            break;
        }
        cursor++;
    }
    if (selected == -1) {
        depletedFlag = true;
    } else {
        elements[selected] = false;
        elements[selected + 1] = true;
        unsigned int count = 0;
        for (int i = selected - 1; i >= 0; i--) {
            if (elements.at(i)) {
                count++;
                elements[i] = false;
            }
        }
        for (unsigned int i = 0; i < count; i++) {
            elements[i] = true;
        }
    }
}

void CombinationGenerator::generateNextAllComb()
{
    unsigned int cursor = 0;
    while ((cursor < elements.size()) && elements.at(cursor)) {
        elements[cursor++] = false;
        countOnes--;
    }
    if (cursor == elements.size()) {
        depletedFlag = true;
    } else {
        elements[cursor] = true;
        countOnes++;
    }

    if (avoidAllOne && (countOnes == elements.size())) {
        depletedFlag = true;
    }
}

void CombinationGenerator::printForDebug(std::string prefix, std::string suffix) const
{
    printf("%s(", prefix.c_str());
    for (unsigned int i = 0; i < elements.size(); i++) {
        printf("%d", (elements.at(i) ? 1 : 0));
        if (i != (elements.size() -1)) {
            printf(" ");
        }
    }
    printf(")%s", suffix.c_str());
}
