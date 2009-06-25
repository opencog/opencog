; opencog/embodiment/scm/predicates-frames.scm
;
; Copyright (C) 2002-2009 Novamente LLC
; All Rights Reserved
; Author(s): Samir Araujo
;
; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU Affero General Public License v3 as
; published by the Free Software Foundation and including the exceptions
; at http://opencog.org/wiki/Licenses
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU Affero General Public License
; along with this program; if not, write to:
; Free Software Foundation, Inc.,
; 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


; Embodiment make use of several AtomTable structures to represent
; perception signals reveived by its sensors, via Proxy.
; Each perception is represented using an EvaluationLink
; and a PredicateNode.
;
; In order to make the Embodiment to use NLP to comunicate
; with an avatar, all these perceptions are needed. However,
; RelEx, the module used to convert sentences said
; by the avatars to Atoms, use Frames to represent what
; was said. So, we need to convert all the Embodiment 
; perceptions into Frames to make them compatible with RelEx.
;
; This files contains a list of all Frames structures, used 
; to represent the Embodiment Perceptions.


; Definition of Frame Relation

; #Relation:Entity1
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Relation")
   (DefinedFrameElementNode "#Relation:Entity1")
)

; #Relation:Entity2
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Relation")
   (DefinedFrameElementNode "#Relation:Entity2")
)

; #Relation:Relation_type
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Relation")
   (DefinedFrameElementNode "#Relation:Relation_type")
)


; Definition of Frame Trajector-Landmark

; #Relation <- #Trajector-Landmark
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Trajector-Landmark")
   (DefinedFrameNode "#Relation")
)

; #Trajector-Landmark:Landmark
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Trajector-Landmark")
   (DefinedFrameElementNode "#Trajector-Landmark:Landmark")
)

; #Trajector-Landmark:Profiled_region
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Trajector-Landmark")
   (DefinedFrameElementNode "#Trajector-Landmark:Profiled_region")
)

; #Trajector-Landmark:Trajector 
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Trajector-Landmark")
   (DefinedFrameElementNode "#Trajector-Landmark:Trajector")
)



; Definition of Frame Locative_relation

; #Trajector-Landmark <- #Locative_relation
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Locative_relation")
   (DefinedFrameNode "#Trajector-Landmark")
)

; #State <- #Locative_relation
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Locative_relation")
   (DefinedFrameNode "#State")
)

; #Trajector-Landmark <- #Locative_relation
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Locative_relation")
   (DefinedFrameNode "#Trajector-Landmark")
)


; #Locative_relation:Figure
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Locative_relation")
   (DefinedFrameElementNode "#Locative_relation:Figure")
)

; #Locative_relation:Ground
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Locative_relation")
   (DefinedFrameElementNode "#Locative_relation:Ground")
)

; an adaptation of the original frame to handle two grounds (between)
; #Locative_relation:Ground_2
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Locative_relation")
   (DefinedFrameElementNode "#Locative_relation:Ground_2")
)


; Definition of Frame Moving_in_place

; #Moving_in_place:Theme
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Moving_in_place")
   (DefinedFrameElementNode "#Moving_in_place:Theme")
)

; #Moving_in_place:Direction
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Moving_in_place")
   (DefinedFrameElementNode "#Moving_in_place:Direction")
)

; #Moving_in_place:Angle
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Moving_in_place")
   (DefinedFrameElementNode "#Moving_in_place:Angle")
)

; #Moving_in_place:Fixed_location
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Moving_in_place")
   (DefinedFrameElementNode "#Moving_in_place:Fixed_location")
)


; Definition of Frame Substance

; #Physical_entity <- #Substance
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Substance")
   (DefinedFrameNode "#Physical_entity")
)


; Definition of Frame Transitive_action

; #Event <- #Transitive_action
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Transitive_action")
   (DefinedFrameNode "#Event")
)

; #Objective_influence <- #Transitive_action
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Transitive_action")
   (DefinedFrameNode "#Objective_influence")
)


; Definition of Frame Intentionally_act

; #Event <- #Intentionally_act
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Intentionally_act")
   (DefinedFrameNode "#Event")
)

; #Transitive_action <- #Intentionally_act
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Intentionally_act")
   (DefinedFrameNode "#Transitive_action")
)



; Definition of Frame Intentionally_affect

; #Intentionally_act <- #Intentionally_affect
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Intentionally_affect")
   (DefinedFrameNode "#Intentionally_act")
)



; Definition of Frame Ingestion

; #Ingest_substance <- #Ingestion
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Ingestion")
   (DefinedFrameNode "#Ingest_substance")
)

; #Manipulation <- #Ingestion
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Ingestion")
   (DefinedFrameNode "#Manipulation")
)

; #Ingestion:Ingestible
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Ingestion")
   (DefinedFrameElementNode "#Ingestion:Ingestible")
)


; Definition of Frame Food

; #Physical_entity <- #Food
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Food")
   (DefinedFrameNode "#Physical_entity")
)


; Definition of Frame Motion

; #Motion:Theme
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion")
   (DefinedFrameElementNode "#Motion:Theme")
)

; #Motion:Direction
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion")
   (DefinedFrameElementNode "#Motion:Direction")
)


; Definition of Frame Dimension

; #Dimension:Dimension
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Dimension")
   (DefinedFrameElementNode "#Dimension:Dimension")
)

; #Dimension:Object 
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Dimension")
   (DefinedFrameElementNode "#Dimension:Object")
)

; #Dimension:Measurement 
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Dimension")
   (DefinedFrameElementNode "#Dimension:Measurement")
)



; Definition of Frame Gradable_attributes

; #Gradable_attributes:Attribute
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Gradable_attributes")
   (DefinedFrameElementNode "#Gradable_attributes:Attribute")
)

; #Gradable_attributes:Degree
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Gradable_attributes")
   (DefinedFrameElementNode "#Gradable_attributes:Degree")
)

; #Gradable_attributes:Value
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Gradable_attributes")
   (DefinedFrameElementNode "#Gradable_attributes:Value")
)




; Definition of Frame Position_on_a_scale

; #Gradable_attributes <- #Position_on_a_scale
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Position_on_a_scale")
   (DefinedFrameNode "#Gradable_attributes")
)

; #Position_on_a_scale:Value
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Position_on_a_scale")
   (DefinedFrameElementNode "#Position_on_a_scale:Value")
)



; Definition of Frame Evaluative_comparison

; #Position_on_a_scale <- #Evaluative_comparison
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Evaluative_comparison")
   (DefinedFrameNode "#Position_on_a_scale")
)


; #Evaluative_comparison:Attribute 
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Evaluative_comparison")
   (DefinedFrameElementNode "#Evaluative_comparison:Attribute")
)

; #Evaluative_comparison:Profiled_attribute
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Evaluative_comparison")
   (DefinedFrameElementNode "#Evaluative_comparison:Profiled_attribute")
)

; #Evaluative_comparison:Profiled_item
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Evaluative_comparison")
   (DefinedFrameElementNode "#Evaluative_comparison:Profiled_item")
)

; #Evaluative_comparison:Standard_attribute
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Evaluative_comparison")
   (DefinedFrameElementNode "#Evaluative_comparison:Standard_attribute")
)

; #Evaluative_comparison:Standard_item
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Evaluative_comparison")
   (DefinedFrameElementNode "#Evaluative_comparison:Standard_item")
)



; Definition of Frame Make_noise

; #Make_noise:Noisy_event
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Make_noise")
   (DefinedFrameElementNode "#Make_noise:Noisy_event")
)

; #Make_noise:Sound
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Make_noise")
   (DefinedFrameElementNode "#Make_noise:Sound")
)

; #Make_noise:Sound_source
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Make_noise")
   (DefinedFrameElementNode "#Make_noise:Sound_source")
)

; #Make_noise:Degree
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Make_noise")
   (DefinedFrameElementNode "#Make_noise:Degree")
)

; #Make_noise:Iterations
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Make_noise")
   (DefinedFrameElementNode "#Make_noise:Iterations")
)



; Definition of Frame Possession

; #Possession:Owner
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Possession")
   (DefinedFrameElementNode "#Possession:Owner")
)

; #Possession:Possession
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Possession")
   (DefinedFrameElementNode "#Possession:Possession")
)



; Definition of Frame Education_teaching

; #Intentionally_act <- #Education_teaching
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameNode "#Intentionally_act")
)


; #Education_teaching:Course
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Course")
)

; #Education_teaching:Fact
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Fact")
)

; #Education_teaching:Institution
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Institution")
)

; #Education_teaching:Material
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Material")
)

; #Education_teaching:Precept
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Precept")
)

; #Education_teaching:Qualification
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Qualification")
)

; #Education_teaching:Role
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Role")
)

; #Education_teaching:Skill
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Skill")
)

; #Education_teaching:Student
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Student")
)

; #Education_teaching:Subject
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Subject")
)

; #Education_teaching:Teacher
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Education_teaching")
   (DefinedFrameElementNode "#Education_teaching:Teacher")
)



; Definition of Frame State

; #State:Entity
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#State")
   (DefinedFrameElementNode "#State:Entity")
)

; #State_of_entity:State
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#State")
   (DefinedFrameElementNode "#State:State")
)



; Definition of Frame State_of_entity

; #State <- #State_of_entity
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#State_of_entity")
   (DefinedFrameNode "#State")
)


; #State_of_entity:Evaluation
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#State_of_entity")
   (DefinedFrameElementNode "#State_of_entity:Evaluation")
)




; Definition of Frame Manipulation

; #Intentionally_affect <- #Manipulation
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Manipulation")
   (DefinedFrameNode "#Intentionally_affect")
)


; #Manipulation:Agent
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Manipulation")
   (DefinedFrameElementNode "#Manipulation:Agent")
)

; #Manipulation:Event
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Manipulation")
   (DefinedFrameElementNode "#Manipulation:Event")
)

; #Manipulation:Bodypart_of_agent
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Manipulation")
   (DefinedFrameElementNode "#Manipulation:Bodypart_of_agent")
)

; #Manipulation:Entity
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Manipulation")
   (DefinedFrameElementNode "#Manipulation:Entity")
)

; #Manipulation:Time
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Manipulation")
   (DefinedFrameElementNode "#Manipulation:Time")
)

; #Manipulation:Duration
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Manipulation")
   (DefinedFrameElementNode "#Manipulation:Duration")
)



; Definition of Frame Motion_directional

; #Motion <- #Motion_directional
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion_directional")
   (DefinedFrameNode "#Motion")
)


; #Motion_directional:Theme
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion_directional")
   (DefinedFrameElementNode "#Motion_directional:Theme")
)

; #Motion_directional:Direction
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion_directional")
   (DefinedFrameElementNode "#Motion_directional:Direction")
)

; #Motion_directional:Goal
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion_directional")
   (DefinedFrameElementNode "#Motion_directional:Goal")
)

; #Motion_directional:Path
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion_directional")
   (DefinedFrameElementNode "#Motion_directional:Path")
)

; #Motion_directional:Source
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Motion_directional")
   (DefinedFrameElementNode "#Motion_directional:Source")
)



; Definition of Frame Obviousness


; #Gradable_attributes <- #Obviousness
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Obviousness")
   (DefinedFrameNode "#Gradable_attributes")
)



; #Obviousness:Attribute
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Obviousness")
   (DefinedFrameElementNode "#Obviousness:Attribute")
)

; #Obviousness:Degree
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Obviousness")
   (DefinedFrameElementNode "#Obviousness:Degree")
)

; #Obviousness:Phenomenon
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Obviousness")
   (DefinedFrameElementNode "#Obviousness:Phenomenon")
)

; #Obviousness:Perceiver
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Obviousness")
   (DefinedFrameElementNode "#Obviousness:Perceiver")
)

; #Obviousness:Location_of_protagonist
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Obviousness")
   (DefinedFrameElementNode "#Obviousness:Location_of_protagonist")
)



; Definition of Frame Biological_urge

; #Gradable_attributes <- #Biological_urge
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Biological_urge")
   (DefinedFrameNode "#Gradable_attributes")
)


; #Biological_urge:Experiencer
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Biological_urge")
   (DefinedFrameElementNode "#Biological_urge:Experiencer")
)

; #Biological_urge:Expressor
(FrameElementLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (DefinedFrameNode "#Biological_urge")
   (DefinedFrameElementNode "#Biological_urge:Expressor")
)





; #Substance <- liquid
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (ConceptNode "liquid")
   (DefinedFrameNode "#Substance")
)

(ReferenceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (WordNode "#liquid")
   (ConceptNode "liquid")
)



; liquid <- water
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (ConceptNode "water")
   (ConceptNode "liquid")
)

(ReferenceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (WordNode "#water")
   (ConceptNode "water")
)



; #Food <- bone
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (ConceptNode "bone")
   (DefinedFrameNode "#Food")
)

(ReferenceLink (stv 1.0 1.0) (cog-new-av 1 0 0)
   (WordNode "#bone")
   (ConceptNode "bone")
)




; The following hierarchy representation aren't frames,
; but will be used to compose perceptions which use frames

; Object <- StaticObject
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "StaticObject")
   (ConceptNode "Object")
)

; StaticObject <- Structure
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "Structure")
   (ConceptNode "StaticObject")
)

; Object <- MovableObject
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "MovableObject")
   (ConceptNode "Object")
)

; MovableObject <- Item
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "Item")
   (ConceptNode "MovableAgent")
)

; MovableObject <- Agent
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "Agent")
   (ConceptNode "MovableAgent")
)

; Agent <- Humanoid
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "Humanoid")
   (ConceptNode "Agent")
)

; Agent <- Pet
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "Pet")
   (ConceptNode "Agent")
)

; Agent <- Avatar
(InheritanceLink (stv 1.0 1.0) (cog-new-av 1 0 0) 
   (ConceptNode "Avatar")
   (ConceptNode "Agent")
)


