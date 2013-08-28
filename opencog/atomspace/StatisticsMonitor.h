/*
 * opencog/atomspace/StatisticsMonitor.h
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

#ifndef _OPENCOG_STATISTICS_MONITOR_H
#define _OPENCOG_STATISTICS_MONITOR_H

#include <time.h>
#include <vector>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * The dynamics statistics agent is responsible for collecting several
 * statistics values from many parts of the system and making them available
 * through its API.
 */
class StatisticsMonitor
{

private:

    int atomCount;
    int nodeCount;
    int linkCount;

    std::vector<int> typeCount;

    float heatSummation;

    std::vector<float> weightSummation;
    std::vector<float> heatTypeSummation;

    std::vector<int> nodeImportanceBinCount;

    /**
     * Updates the number of atoms of a given type, as well as the total
     * atom count.
     *
     * @param Atom type count to be updated.
     * @param Number of atoms to be incremented to the given type count
     * (decrements if negative).
     */
    void updateTypeCount(Type, int);

public:

    // JUST FOR TESTS
    void init();
    bool isCleared();

    /**
     * Constructor for this class.
     */
    StatisticsMonitor();

    /**
     * Returns singleton instance.
     *
     * @return Singleton instance.
     */
    static StatisticsMonitor* getInstance();

    /**
     * Returns the current lobe cycle.
     *
     * @return Current lobe cycle.
     */
    int getLobeCycle();

    /**
     * This method is called whenever the agent is executed by the
     * scheduler. Since this agent is supposed to be executed once every
     * cycle, this method keeps track of what is the current Lobe cycle.
     */
    void execute();

    /**
     * Updates the total weight summation of all atoms and of atoms of a
     * given type.
     *
     * @param Atom type to be updated.
     * @param Value to be added to the total weight summation for both the
     * given type and all atoms.
     */
    void updateWeightSummation(Type, float);

    /**
     * Returns the mean weight for atoms of a given type.
     *
     * @return Mean weight for atoms of a given type.
     */
    float getMeanWeight(Type);

    /**
     * Updates the total heat summation of all atoms and of atoms of a
     * given type.
     *
     * @param Atom type to be updated.
     * @param Value to be added to the total heat summation for both the
     * given type and all atoms.
     */
    void updateHeatSummation(Type, float);

    /**
     * Returns the mean heat for all atoms.
     *
     * @return Mean heat for all atoms.
     */
    float getMeanHeat();

    /**
     * Returns the mean heat for all atoms of a given type.
     *
     * @return Mean heat for all atoms of a given type.
     */
    float getMeanHeat(Type);

    /**
     * Updates all statistics from scratch. This is expensive since all
     * atoms in the system must be traversed.
     */
    void reevaluateAllStatistics(const AtomSpace&);

    /**
     * Returns the total number of atoms in the system.
     *
     * @return Total number of atoms in the system.
     */
    int getAtomCount();

    /**
     * Returns the total number of nodes in the system.
     *
     * @return Total number of nodes in the system.
     */
    int getNodeCount();

    /**
     * Returns the total number of links in the system.
     *
     * @return Total number of links in the system.
     */
    int getLinkCount();

    /**
     * Changes importance statistics whenever an atom changes importance
     * bins.
     *
     * @param Atom type that changed importance bin.
     * @param Old importance bin.
     * @param New importance bin.
     */
    void atomChangeImportanceBin(Type, int, int);

    /**
     * Returns the number of nodes in a given importance bin.
     *
     * @param Importance bin.
     * @return Number of nodes in a given importance bin.
     */
    int getNodeImportanceBinCount(int);

    /**
     * Updates all necessary statistics whenever a new atom is added to
     * the system.
     *
     * @param Atom being added.
     */
    void add(Atom*);

    /**
     * Updates all necessary statistics whenever an atom is removed from
     * the system.
     *
     * @param Atom being removed.
     */
    void remove(Atom*);

    int getTypeCount(Type type);

};

/** @}*/
} // namespace opencog

#endif // _OPENCOG_STATISTICS_MONITOR_H
