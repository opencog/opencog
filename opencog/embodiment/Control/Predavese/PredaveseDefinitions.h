/*
 * opencog/embodiment/Control/Predavese/PredaveseDefinitions.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

/**
 * Predavese definitions that should be included only in Predavese source (*.cc) files, not in other headers.
 */

// 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79,
// 83, 89, 97, 101, 103, 107, 109, 113

#define Elem                2

#define Relation            2*3
#define Relation_obj        2*3*5
#define Relation_subj        2*3*7
#define Relation_obj2        2*3*11
#define Relation_poss        2*3*13
#define Relation_subj_r        2*3*17
#define Relation_to            2*3*19
#define Relation_to_do        2*3*23

#define Term                2*29

#define BasicTerm            2*29*31
#define Verb                2*29*31*37
#define GeneralWord            2*29*31*41

#define Name                2*29*31*43
#define ProperName            2*29*31*43*47
#define TrickName            2*29*31*43*53
#define PetName                2*29*31*43*73

#define Adj                 2*29*31*59
#define Pron                2*29*31*67
#define Poss                2*29*31*71

#define QualifiedBasicTerm    2*29*61

#define KeyVerb                2*5

#define MetaCommand         2*7

// Update this whenever you add in new types:

#define NEXT_FREE_TERMINAL_ID (PetName+1)
