#ifndef _TEMPORAL_RELATED_TESTS_COMMONS_H_ 
#define _TEMPORAL_RELATED_TESTS_COMMONS_H_ 
/*
 * tests/atomspace/TemporalRelatedTestsConstants.h
 * Common declarations used in unit tests related to Temporal relationship search/removal criteria
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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
 *
 */

#define N_TIMES 10
#define NUMBER_OF_HANDLES 2
#define NUMBER_OF_SEARCH_INTERVALS 10
#define NUMBER_OF_CRITERIA 11

    static Temporal search_intervals[NUMBER_OF_SEARCH_INTERVALS] = {
          Temporal(0,0), 
          Temporal(0,1),
          Temporal(1,1), 
          Temporal(1,2),
          Temporal(2,2), 
          Temporal(2,3),
          Temporal(3,3), 
          Temporal(3,4),
          Temporal(4,4), 
          Temporal(4,5)
        };

    static TemporalTable::TemporalRelationship criteria[NUMBER_OF_CRITERIA] = {
            TemporalTable::STARTS_BEFORE,
            TemporalTable::STARTS_WITHIN,
            TemporalTable::STARTS_AFTER,
            TemporalTable::ENDS_BEFORE,
            TemporalTable::ENDS_WITHIN,
            TemporalTable::ENDS_AFTER,
            TemporalTable::NEXT_AFTER_START_OF,
            TemporalTable::NEXT_AFTER_END_OF,
            TemporalTable::PREVIOUS_BEFORE_START_OF,
            TemporalTable::PREVIOUS_BEFORE_END_OF,
            TemporalTable::INCLUDES,
        };
        
    /* Map of entries for these tests: 
     *  H0       H1
     * [0,0]    [1,1]
     * [1,2]    [2,3]
     * [0,2]    [1,3]
     */
    static int expectedNumberOfEntries[NUMBER_OF_HANDLES][NUMBER_OF_SEARCH_INTERVALS][NUMBER_OF_CRITERIA] = {
         { {0,2,1,0,1,2,1,1,0,0,2},{0,3,0,0,1,2,1,0,0,1,1},{2,1,0,1,0,2,0,0,1,1,2},{2,1,0,1,2,0,0,0,1,1,2},{3,0,0,1,2,0,0,0,1,1,2},
           {3,0,0,1,2,0,0,0,1,1,0},{3,0,0,3,0,0,0,0,1,1,0},{3,0,0,3,0,0,0,0,1,1,0},{3,0,0,3,0,0,0,0,1,1,0},{3,0,0,3,0,0,0,0,1,1,0} },
         { {0,0,3,0,0,3,1,1,0,0,0},{0,2,1,0,1,2,1,1,0,0,0},{0,2,1,0,1,2,1,1,0,0,2},{0,3,0,0,1,2,1,0,0,1,1},{2,1,0,1,0,2,0,0,1,1,2},
           {2,1,0,1,2,0,0,0,1,1,2},{3,0,0,1,2,0,0,0,1,1,2},{3,0,0,1,2,0,0,0,1,1,0},{3,0,0,3,0,0,0,0,1,1,0},{3,0,0,3,0,0,0,0,1,1,0} }
        };
    static Temporal expectedNextAfterStartOf[NUMBER_OF_HANDLES][NUMBER_OF_SEARCH_INTERVALS] = {
         {Temporal(1,2),Temporal(1,2),UNDEFINED_TEMPORAL, UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL},
         {Temporal(1,1),Temporal(1,1),Temporal(2,3),Temporal(2,3),UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL}
        };
    static Temporal expectedNextAfterEndOf[NUMBER_OF_HANDLES][NUMBER_OF_SEARCH_INTERVALS] = {
         {Temporal(1,2),UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL, UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL},
         {Temporal(1,1),Temporal(2,3),Temporal(2,3),UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL}
        };
    static Temporal expectedPreviousBeforeStartOf[NUMBER_OF_HANDLES][NUMBER_OF_SEARCH_INTERVALS] = {
         {UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,Temporal(0,2),Temporal(0,2),Temporal(1,2),Temporal(1,2),Temporal(1,2),Temporal(1,2),Temporal(1,2),Temporal(1,2)},
         {UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,Temporal(1,3),Temporal(1,3),Temporal(2,3),Temporal(2,3),Temporal(2,3),Temporal(2,3)}
        };
    static Temporal expectedPreviousBeforeEndOf[NUMBER_OF_HANDLES][NUMBER_OF_SEARCH_INTERVALS] = {
         {UNDEFINED_TEMPORAL,Temporal(0,2),Temporal(0,2),Temporal(1,2),Temporal(1,2),Temporal(1,2),Temporal(1,2),Temporal(1,2),Temporal(1,2),Temporal(1,2)},
         {UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,UNDEFINED_TEMPORAL,Temporal(1,3),Temporal(1,3),Temporal(2,3),Temporal(2,3),Temporal(2,3),Temporal(2,3),Temporal(2,3)}
        };

#endif //_TEMPORAL_RELATED_TESTS_COMMONS_H_ 
