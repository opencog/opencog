#ifndef _ATOM_SPACE_DEFINITIONS_H_
#define _ATOM_SPACE_DEFINITIONS_H_
/**
 * AtomSpaceDefinitions.h
 * 
 * A special header file to put all definitions needed by AtomSpace's classes.
 * 
 */

// Atom flags

#define WRITE_MUTEX             1  //BIT0
#define MARKED_FOR_REMOVAL      2  //BIT1
//#define MULTIPLE_TRUTH_VALUES 4  //BIT2
#define FIRED_ACTIVATION        8  //BIT3
#define HYPOTETHICAL_FLAG       16 //BIT4

// AtomTable indices 

#define TYPE_INDEX          0
#define NAME_INDEX          1
#define IMPORTANCE_INDEX    2
#define NUMBER_OF_INDICES   3
#define TARGET_TYPE_INDEX   (1UL << 16)
#define PREDICATE_INDEX     (1UL << 17)

#define NAME_INDEX_SIZE         (1 << 16)
#define IMPORTANCE_INDEX_SIZE   (1 << 16)
#define MAX_PREDICATE_INDICES   32

#endif // _ATOM_SPACE_DEFINITIONS_H_
