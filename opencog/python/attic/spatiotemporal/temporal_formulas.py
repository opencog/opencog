from temporal_interval_handling import getBeginning, getEnding, getSize, calculateCenterMass, normalize

#Extensions:
#-Implement as fuzzy operators the ANDs
#-Calculate a fuzzy value by considering the distance between both. In this case it may
#be interesting to take into account the total length of the beginning and/or ending
#-Deal with fuzzy equality. 

#Formulas:
#temporal relations are calculated by considering the CoM of the beginning and/or the ending
#of the two temporal intervals. In this first and simple version only the temporal aspect 
#of the CoM is taken into account, regardless of the certainty. Accordingly, the input 
#of each function is the two timestamps.
#The order for the input parameters are these: <a-,a+,b-,b+>

#Improvements for future versions:
#-Consider as well the certainty of the CoM
#of both subintervals and ponderate accordingly



#totallength = (secondkey1-firstkey1) + (secondkey2-firstkey2)

#com_beg_1 < com_beg_2 and com_beg_2 < com_end_1
#min(a,b)?
#max(0,a+b-1)?

#return com_end_1 == com_beg_2
# (totallength - (|com_end_1 - com_beg_2|) ) / totallength


def beforeFormula(dist1, dist2):
#a+ < b-
    end_1 = getEnding(dist1)
    beg_2 = getBeginning(dist2)

    com_end_1 = calculateCenterMass(end_1)
    com_beg_2 = calculateCenterMass(beg_2)

    size_end_1 = getSize(end_1)
    size_beg_2 = getSize(beg_2)

    #return com_end_1 < com_beg_2
    return normalize((com_beg_2[0] - com_end_1[0]) / (size_end_1 + size_beg_2))


def overlapsFormula(dist1, dist2):
#a- < b- and b- < a+ and a+ < b+
    beg_1 = getBeginning(dist1)
    end_1 = getEnding(dist1)
    beg_2 = getBeginning(dist2)
    end_2 = getEnding(dist2)

    com_beg_1 = calculateCenterMass(beg_1)
    com_end_1 = calculateCenterMass(end_1)
    com_beg_2 = calculateCenterMass(beg_2)
    com_end_2 = calculateCenterMass(end_2)

    size_beg_1 = getSize(beg_1)
    size_end_1 = getSize(end_1)
    size_beg_2 = getSize(beg_2)
    size_end_2 = getSize(end_2)

    return normalize(min((com_beg_2[0] - com_beg_1[0]) / (size_beg_2 + size_beg_1),
                         (com_end_1[0] - com_beg_2[0]) / (size_end_1 + size_beg_2),
                         (com_end_2[0] - com_end_1[0]) / (size_end_2 + size_end_1)))


def duringFormula(dist1, dist2):
#b- < a- and a+ < b+
    beg_1 = getBeginning(dist1)
    end_1 = getEnding(dist1)
    beg_2 = getBeginning(dist2)
    end_2 = getEnding(dist2)

    com_beg_1 = calculateCenterMass(beg_1)
    com_end_1 = calculateCenterMass(end_1)
    com_beg_2 = calculateCenterMass(beg_2)
    com_end_2 = calculateCenterMass(end_2)

    size_beg_1 = getSize(beg_1)
    size_end_1 = getSize(end_1)
    size_beg_2 = getSize(beg_2)
    size_end_2 = getSize(end_2)

    return normalize(min((com_beg_1[0] - com_beg_2[0]) / (size_beg_1 + size_beg_2),
                         (com_end_2[0] - com_end_1[0]) / (size_end_2 + size_end_1)))


def meetsFormula(dist1, dist2):
#a+ = b-
    end_1 = getEnding(dist1)
    beg_2 = getBeginning(dist2)

    com_end_1 = calculateCenterMass(end_1)
    com_beg_2 = calculateCenterMass(beg_2)

    size_end_1 = getSize(end_1)
    size_beg_2 = getSize(beg_2)

    return normalize(((size_end_1 + size_beg_2) - abs(com_end_1[0] - com_beg_2[0]))
                     / (size_end_1 + size_beg_2))


def startsFormula(dist1, dist2):
#a- = b- and a+ < b+
    beg_1 = getBeginning(dist1)
    end_1 = getEnding(dist1)
    beg_2 = getBeginning(dist2)
    end_2 = getEnding(dist2)

    com_beg_1 = calculateCenterMass(beg_1)
    com_end_1 = calculateCenterMass(end_1)
    com_beg_2 = calculateCenterMass(beg_2)
    com_end_2 = calculateCenterMass(end_2)

    size_beg_1 = getSize(beg_1)
    size_end_1 = getSize(end_1)
    size_beg_2 = getSize(beg_2)
    size_end_2 = getSize(end_2)

    print("Beg1: {0}, end1: {1}".format(beg_1, end_1))
    print("Beg2: {0}, end2: {1}".format(beg_2, end_2))
    print("{0}, {1}".format(dist1, dist2))
    print(size_beg_1 + size_beg_2)
    print(size_end_2 + size_end_1)

    return normalize(min(((size_beg_1 + size_beg_2) - abs(com_beg_1[0] - com_beg_2[0]))
                         / (size_beg_1 + size_beg_2),
                         (com_end_2[0] - com_end_1[0]) / (size_end_2 + size_end_1)))


def finishesFormula(dist1, dist2):
#a+ = b+ and b- < a-
    beg_1 = getBeginning(dist1)
    end_1 = getEnding(dist1)
    beg_2 = getBeginning(dist2)
    end_2 = getEnding(dist2)

    com_beg_1 = calculateCenterMass(beg_1)
    com_end_1 = calculateCenterMass(end_1)
    com_beg_2 = calculateCenterMass(beg_2)
    com_end_2 = calculateCenterMass(end_2)

    size_beg_1 = getSize(beg_1)
    size_end_1 = getSize(end_1)
    size_beg_2 = getSize(beg_2)
    size_end_2 = getSize(end_2)

    print size_beg_1
    print size_beg_2
    print size_end_1
    print size_end_2

    return normalize(min(((size_end_1 + size_end_2) - abs(com_end_1[0] - com_end_2[0]))
                         / (size_end_1 + size_end_2),
                         (com_beg_1[0] - com_beg_2[0]) / (size_beg_1 + size_beg_2)))


def equalsFormula(dist1, dist2):
#a- = b- and a+ = b+
    beg_1 = getBeginning(dist1)
    end_1 = getEnding(dist1)
    beg_2 = getBeginning(dist2)
    end_2 = getEnding(dist2)

    com_beg_1 = calculateCenterMass(beg_1)
    com_end_1 = calculateCenterMass(end_1)
    com_beg_2 = calculateCenterMass(beg_2)
    com_end_2 = calculateCenterMass(end_2)

    size_beg_1 = getSize(beg_1)
    size_end_1 = getSize(end_1)
    size_beg_2 = getSize(beg_2)
    size_end_2 = getSize(end_2)

    return normalize(
        min(((size_beg_1 + size_beg_2) - abs(com_beg_1[0] - com_beg_2[0])) / (size_beg_1 + size_beg_2),
            ((size_end_1 + size_end_2) - abs(com_end_1[0] - com_end_2[0])) / (size_end_1 + size_end_2)))


def afterFormula(dist1, dist2):
#before(Y,X)
    return beforeFormula(dist2, dist1)


def overlapped_byFormula(dist1, dist2):
#overlaps(Y,X)
    return overlapsFormula(dist2, dist1)


def containsFormula(dist1, dist2):
#during(Y,X)
    return duringFormula(dist2, dist1)


def met_byFormula(dist1, dist2):
#meets(Y,X)
    return meetsFormula(dist2, dist1)


def started_byFormula(dist1, dist2):
#starts(Y,X)
    return startsFormula(dist2, dist1)


def finished_byFormula(dist1, dist2):
#finishes(Y,X)
    return finishesFormula(dist2, dist1)
