#include "PartitionGenerator.h"

using namespace opencog;

PartitionGenerator::PartitionGenerator(unsigned int n, bool includeTrivial) 
{
    if (n == 0) {
        throw std::runtime_error("Error creating PartitionGenerator\n");
    }

    if (n > 1) {
        computePartitions(partitions, n);
    }

    if (includeTrivial) {
        IntegerSetSet partition;
        IntegerSet component;
        for (unsigned int i = 0; i < n; i++) {
            component.insert(i);
        }
        partition.insert(component);
        partitions.insert(partition);
    }
    partitionIterator = partitions.begin();
}

PartitionGenerator::~PartitionGenerator() 
{
}

bool PartitionGenerator::depleted() const
{
    return partitionIterator == partitions.end();
}

void PartitionGenerator::generateNext()
{
    if (depleted()) {
        throw std::runtime_error("PartitionGenerator depleted\n");
    }

    ++partitionIterator;
}

PartitionGenerator::IntegerSetSet PartitionGenerator::getPartition() const
{
    return *partitionIterator;
}

void PartitionGenerator::printForDebug(std::string prefix, std::string suffix) const
{
    printf("%s{", prefix.c_str());
    unsigned int count1 = 0;
    for (const IntegerSet& comp : *partitionIterator) {
        printf("{");
        unsigned int count2 = 0;
        for (unsigned int el : comp) {
            printf("%u", el);
            if (count2++ != (comp.size() - 1)) {
                printf(" ");
            }
        }
        printf("}");
        if (count1++ != ((*partitionIterator).size() - 1)) {
            printf(" ");
        }
    }
    printf("}%s", suffix.c_str());
}

void PartitionGenerator::computePartitions(IntegerSetSetSet &answer, unsigned int n)
{
    answer.clear();
    IntegerSetSet partition;
    IntegerSet component;

    if (n == 1) {
        component.insert(0);
        partition.insert(component);
        answer.insert(partition);
    } else {
        IntegerSetSetSet recurseAnswer;
        for (unsigned int i = 0; i < n; i++) {
            partition.clear();
            component.clear();
            component.insert(i);
            partition.insert(component);
            component.clear();
            for (unsigned int j = 0; j < n; j++) {
                if (j != i) {
                    component.insert(j);
                }
            }
            partition.insert(component);
            answer.insert(partition);
            recurseAnswer.clear();
            computePartitions(recurseAnswer, n - 1);
            for (const IntegerSetSet& part : recurseAnswer) {
                partition.clear();
                component.clear();
                component.insert(i);
                partition.insert(component);
                for (const IntegerSet& comp : part) {
                    component.clear();
                    for (unsigned int el : comp) {
	                    component.insert(el + (el < i ? 0 : 1));
                    }
                    partition.insert(component);
                }
                answer.insert(partition);
            }
        }
    }
}
