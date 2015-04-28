#!/usr/bin/env tclsh 


# To run this script just execute it in the command line. It will
# create BasicSchemaFeeder.cc and BasicPredicateFeeder.cc in
# the current directory.

# this is not actually required to run this script. It's here for convenience
# as behavior names are required to build BasicSchemaFeeder.cc
set behaviorNames [list grab_nearest sit_small_object rotate_lie_down come_with_owner heel_sit wander_searching_food wander_searching_water pick_carry_drop_chew_grab pick_carry_drop pick_chew pick_grabs_scratch wander_searching_pee_place wander_searching_poo_place pay_attention follow_owner seek_food_whine sniff_butt chase nudge_around bring_to_owner bark_non_friendly_avatar bark_non_friendly_pet lick_friendly_avatar greet_avatar_with_jump sleep_near_owner sit_near_owner_and_stay followOwner beg eat drink nap sit goto_obj bark bite flee lick sniff jump play wander poo pee full_of_doubts goto_random_pickupable goto_random_edible goto_random_object goto_random_drinkable goto_random_avatar goto_random_pet goto_random_small goto_random_moving goto_random_poo_place goto_random_pee_place gonear_obj goto_nearest_and_grabit]
# The set bellow is for Multiverse (TODO: check if the rule engine functions must be included here too)
#set behaviorNames [list grab_nearest sit_small_object come_with_owner wander_searching_food wander_searching_water pick_carry_drop wander_searching_pee_place wander_searching_poo_place follow_owner seek_food_whine bring_to_owner bark_non_friendly_avatar sleep_near_owner sit_near_owner_and_stay]
#set behaviorNames [list test_function]


################################################################################
# procedures definition

proc fillMatrixWithSymetricElements {predicates} {

    global similarity

    for {set i 0} {$i < [llength $predicates]} {incr i} {
        for {set j 0} {$j < [llength $predicates]} {incr j} {
            if {$i == $j} {
                set similarity([lindex $predicates $i],[lindex $predicates $j]) 1.0
            } else {
                if {$i > $j} {
                    set similarity([lindex $predicates $i],[lindex $predicates $j]) $similarity([lindex $predicates $j],[lindex $predicates $i])
                }
            }
        }
    }
}

proc setPropositionDefaults {} {

    global similarity propositions

    set propositions [list is_avatar is_movable is_pet]

    set similarity(is_avatar,is_movable) 0.25
    set similarity(is_avatar,is_pet) 0.60
    set similarity(is_movable,is_pet) 0.25
}

proc setRelationshipsDefaults {} {

    global similarity relationships

    set relationships [list near above below beside inside outside]

    set similarity(near,above) 0.25
    set similarity(near,below) 0.25
    set similarity(near,beside) 0.60
    set similarity(near,inside) 0.25
    set similarity(near,outside) 0.15
    set similarity(above,below) 0.60
    set similarity(above,beside) 0.25
    set similarity(above,inside) 0.15
    set similarity(above,outside) 0.25
    set similarity(below,beside) 0.25
    set similarity(below,inside) 0.15
    set similarity(below,outside) 0.25
    set similarity(beside,inside) 0.15
    set similarity(beside,outside) 0.30
    set similarity(inside,outside) 0.60
}

proc setCategoriesDefaults {} {

    global actions inherits broadCategories narrowCategories extremelyNarrowCategories

    set broadCategories [list movement local_activity social]
    set narrowCategories [list always_local_movement potentially_long_range_movement bodily_gesture object_manipulation communication]
    set extremelyNarrowCategories [list sound]

    foreach category1 $extremelyNarrowCategories {
        foreach category2 $narrowCategories {
            set inherits($category1,$category2) 0
        }
    }

    foreach category1 $narrowCategories {
        foreach category2 $broadCategories {
            set inherits($category1,$category2) 0
        }
    }

    set inherits(always_local_movement,movement) 1
    set inherits(potentially_long_range_movement,movement) 1
    set inherits(bodily_gesture,local_activity) 1
}

proc setActionsDefaults {} {
    
    global actions inherits broadCategories narrowCategories extremelyNarrowCategories categories isAntonymic

    # basics

    #set actions [list goto_obj grab_obj step_forward step_backward rotate_left rotate_right jump drop sniff bark howl bare_teeth fart wag stretch sit beg heel random_step grab_nearest]
    
    # old actions list... now all actions are uppercase
    #set actions [list grab jump drop sniff bark howl bareTeeth wagTail stretch sit beg heel]
    set actions [list GRAB JUMP DROP SNIFF BARK HOWL BARETEETH WAGTAIL STRETCH SIT BEG HEEL PAYATTENTION GRAB_NEAREST]

    # categories

    #set categories(goto_obj) [list potentially_long_range_movement]
    #set categories(step_forward) [list always_local_movement]
    #set categories(step_backward) [list always_local_movement]
    #set categories(rotate_left) [list always_local_movement local_activity]
    #set categories(rotate_right) [list always_local_movement local_activity]
    ##set categories(jump) [list always_local_movement]
    ##set categories(drop) [list object_manipulation]
    ##set categories(grab) [list object_manipulation bodily_gesture]
    ##set categories(sniff) [list bodily_gesture]
    ##set categories(bark) [list bodily_gesture sound communication]
    ##set categories(howl) [list bodily_gesture sound communication]
    ##set categories(bareTeeth) [list bodily_gesture communication]
    #set categories(fart) [list bodily_gesture]
    ##set categories(wagTail) [list bodily_gesture communication]
    ##set categories(stretch) [list bodily_gesture]
    ##set categories(sit) [list bodily_gesture]
    ##set categories(beg) [list bodily_gesture communication]
    ##set categories(heel) [list potentially_long_range_movement social]
    #set categories(random_step) [list potentially_long_range_movement]
    #set categories(grab_nearest) [list object_manipulation bodily_gesture]

    set categories(JUMP)  [list always_local_movement]
    set categories(DROP)  [list object_manipulation]
    set categories(GRAB)  [list object_manipulation bodily_gesture]
    set categories(GRAB_NEAREST)  [list object_manipulation bodily_gesture]
    set categories(SNIFF) [list bodily_gesture]
    set categories(BARK)  [list bodily_gesture sound communication]
    set categories(HOWL)  [list bodily_gesture sound communication]
    set categories(SIT)   [list bodily_gesture]
    set categories(BEG)   [list bodily_gesture communication]
    set categories(HEEL)  [list potentially_long_range_movement social]
    set categories(WAGTAIL) [list bodily_gesture communication]
    set categories(STRETCH) [list bodily_gesture]
    set categories(BARETEETH) [list bodily_gesture communication]
    set categories(PAYATTENTION)   [list bodily_gesture]

    # inherited categories

    foreach predicate $actions {
        if {[lsearch $actions $predicate] == -1} {
            puts "Unknown predicate <$predicate>"
            exit
        }
        foreach category $categories($predicate) {
            if {([lsearch $broadCategories $category] == -1) && ([lsearch $narrowCategories $category] == -1) && ([lsearch $extremelyNarrowCategories $category] == -1)} {
                puts "Unknown category <$category>"
                exit
            }
            if {[lsearch $extremelyNarrowCategories $category] != -1} {
                foreach superCategory $narrowCategories {
                    if {($inherits($category,$superCategory)) && ([lsearch $categories($predicate) $superCategory] == -1)} {
                        lappend categories($predicate) $superCategory
                        #puts "lappend categories($predicate) $superCategory"
                    }
                }
            } else {
                if {[lsearch $narrowCategories $category] != -1} {
                    foreach superCategory $broadCategories {
                        if {($inherits($category,$superCategory)) && ([lsearch $categories($predicate) $superCategory] == -1)} {
                            lappend categories($predicate) $superCategory
                            #puts "lappend categories($predicate) $superCategory"
                        }
                    }
                }
            }
        }
    }

    # antonymics

    foreach a1 $actions {
        foreach a2 $actions {
            set isAntonymic($a1,$a2) 0
        }
    }

    #set isAntonymic(step_forward,step_backward) 1
    #set isAntonymic(step_backward,step_forward) 1
    #set isAntonymic(rotate_left,rotate_right) 1
    #set isAntonymic(rotate_right,rotate_left) 1
    set isAntonymic(grab,drop) 1
    set isAntonymic(drop,grab) 1
    #set isAntonymic(grab_nearest,drop) 1
    #set isAntonymic(drop,grab_nearest) 1
}

proc computePairwiseSimilarity {pred1 pred2} {

    global isAntonymic categories broadCategories narrowCategories extremelyNarrowCategories

    set matched {}

    if {$isAntonymic($pred1,$pred2)} {
        lappend matched 0.60
    }

    set cat1 $categories($pred1)
    set cat2 $categories($pred2)

    set intersectionExtremelyNarrow 0
    foreach c $narrowCategories {
        if {([lsearch $cat1 $c] != -1) && ([lsearch $cat2 $c] != -1)} {
            incr intersectionExtremelyNarrow
        }
    }

    set intersectionNarrow 0
    foreach c $narrowCategories {
        if {([lsearch $cat1 $c] != -1) && ([lsearch $cat2 $c] != -1)} {
            incr intersectionNarrow
        }
    }

    set intersectionBroad 0
    foreach c $broadCategories {
        if {([lsearch $cat1 $c] != -1) && ([lsearch $cat2 $c] != -1)} {
            incr intersectionBroad
        }
    }


    if {$intersectionNarrow > 0} {
        if {$intersectionNarrow > 1} {
            lappend matched 0.50
        } else {
            lappend matched 0.40
        }
    } else {
        if {$intersectionBroad > 0} {
            lappend matched 0.25
        }
    }

    if {$intersectionExtremelyNarrow > 0} {
        if {$intersectionNarrow > 1} {
            lappend matched 0.70
        } else {
            lappend matched 0.50
        }
    }
    
    set answer 0.00
    foreach a $matched {
        if {$a > $answer} {
            set answer $a
        }
    }

    return $answer
}

proc computeSimilarityOfActions {} {

    global similarity actions

    for {set i 0} {$i < [llength $actions]} {incr i} {
        for {set j [expr $i + 1]} {$j < [llength $actions]} {incr j} {
            set similarity([lindex $actions $i],[lindex $actions $j]) [computePairwiseSimilarity [lindex $actions $i] [lindex $actions $j]]
        }
    }
}

proc printMatrix {predicates} {

    global similarity

    set line [list {}]
    foreach p $predicates {
        lappend line $p
    }
    puts [join $line ","]
    foreach p1 $predicates {
        set line [list $p1]
        foreach p2 $predicates {
            lappend line $similarity($p1,$p2)
        }
        puts [join $line ","]
    }
}

proc emitSchemaCPPCode {} {

    global actions similarity behaviorNames

    set prefix "/**
 * BasicSchemaFeeder.cc
 *
 * \$Header\$
 *
 * Author: Automatically generated by buildSimilarityLinksCode.tcl
 */

// ********************************************************************************
// ***   WARNNING
// ********************************************************************************
//
// This file is generated automatically using buildSimilarityLinksCode.tcl
// Please do not change it by hand.
//
// ********************************************************************************

#include \"BasicSchemaFeeder.h\"
#include <vector>

using namespace OperationalAvatarController;

BasicSchemaFeeder::~BasicSchemaFeeder() {
}

BasicSchemaFeeder::BasicSchemaFeeder() {
}

void BasicSchemaFeeder::populateAtomSpaceWithInitialAtoms(GoalsActionsImportance *gai) {

    std::vector<Handle> schemaNodes;
"

    set suffix "}"

    set f [open "BasicSchemaFeeder.cc" w]
    puts $f $prefix
    foreach a $actions {
        puts $f [format "    schemaNodes.push_back(gai->addSchema(\"$a\", false));"]
    }
    puts $f "\n    // Behaviors (combo). No similarity links between them."
    foreach a $behaviorNames {
        puts $f [format "    schemaNodes.push_back(gai->addSchema(\"$a\", true));"]
    }
    puts $f ""
    for {set i 0} {$i < [llength $actions]} {incr i} {
        for {set j [expr $i + 1]} {$j < [llength $actions]} {incr j} {
            puts $f [format "    gai->addSimilarityLink(schemaNodes\[$i\], schemaNodes\[$j\], $similarity([lindex $actions $i],[lindex $actions $j]));"]
        }
    }
    puts $f $suffix
    close $f

}

proc emitPredicateCPPCode {} {

    global propositions relationships similarity
    set allPredicates [concat $propositions $relationships]

    set prefix "/**
 * BasicPredicateFeeder.cc
 *
 * \$Header\$
 *
 * Author: Automatically generated by buildSimilarityLinksCode.tcl
 */

// ********************************************************************************
// ***   WARNNING
// ********************************************************************************
//
// This file is generated automatically using buildSimilarityLinksCode.tcl
// Please do not change it by hand.
//
// ********************************************************************************

#include \"BasicPredicateFeeder.h\"
#include <Link.h>
#include <SimpleTruthValue.h>
#include <vector>

using namespace OperationalAvatarController;

BasicPredicateFeeder::~BasicPredicateFeeder() {
}

BasicPredicateFeeder::BasicPredicateFeeder() {
}

Handle BasicPredicateFeeder::addSimilarityLink(GoalsActionsImportance *gai, Handle h1, Handle h2, float strength) {

    HandleSeq handles(2);
    handles\[0\] = h1;
    handles\[1\] = h2;
    Link *link = ((Link *) TLB::getAtom(gai->addLink(SIMILARITY_LINK, handles)));
    SimpleTruthValue newTruthValue(strength, 18); // MAGIC NUMBER SUGGESTED BY CASSIO TO MAKE CONFIDENCE = 0.9
    link->setTruthValue(newTruthValue);

    return link;
}


void BasicPredicateFeeder::populateAtomSpaceWithInitialAtoms(GoalsActionsImportance *gai) {

    std::vector<Handle> nodes;
"

    set suffix "}"

    set f [open "BasicPredicateFeeder.cc" w]
    puts $f $prefix
    foreach a $allPredicates {
        puts $f [format "    nodes.push_back(gai->addNode(GROUNDED_PREDICATE_NODE, \"$a\"));"]
    }
    puts $f ""
    for {set i 0} {$i < [llength $allPredicates]} {incr i} {
        for {set j [expr $i + 1]} {$j < [llength $allPredicates]} {incr j} {
            puts $f [format "    addSimilarityLink(gai, nodes\[$i\], nodes\[$j\], $similarity([lindex $allPredicates $i],[lindex $allPredicates $j]));"]
        }
    }
    puts $f $suffix
    close $f

}

# end of procedures definition
################################################################################

# propositions similarity

setPropositionDefaults
fillMatrixWithSymetricElements $propositions 

# relationships similarity

setRelationshipsDefaults
fillMatrixWithSymetricElements $relationships 

foreach p1 $propositions {
    foreach p2 $relationships {
        set similarity($p1,$p2) 0.0
        set similarity($p2,$p1) 0.0
    }
}

# actions similarity

setCategoriesDefaults
setActionsDefaults
computeSimilarityOfActions
fillMatrixWithSymetricElements $actions

# debug printing
#printMatrix $propositions
#printMatrix $relationships
#printMatrix $actions

emitSchemaCPPCode 
emitPredicateCPPCode 


