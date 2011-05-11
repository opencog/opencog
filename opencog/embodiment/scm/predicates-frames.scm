; opencog/embodiment/scm/predicates-frames.scm
;
; Copyright (C) 2009 Novamente LLC
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


; Embodiment make use of several AtomSpace structures to represent
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
; This files contains a list of Frames structures used 
; to represent some custom (extends Framenet) Embodiment Perceptions.

; #Substance <- liquid
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0)
   (ConceptNode "liquid")
   (DefinedFrameNode "#Substance")
)

(ReferenceLink (stv 1.0 1.0) (cog-new-av 0 1 0)
   (WordNode "#liquid")
   (ConceptNode "liquid")
)

; liquid <- water
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0)
   (ConceptNode "water")
   (ConceptNode "liquid")
)

(ReferenceLink (stv 1.0 1.0) (cog-new-av 0 1 0)
   (WordNode "#water")
   (ConceptNode "water")
)



; #Food <- bone
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0)
   (ConceptNode "bone")
   (DefinedFrameNode "#Food")
)

(ReferenceLink (stv 1.0 1.0) (cog-new-av 0 1 0)
   (WordNode "#bone")
   (ConceptNode "bone")
)




; The following hierarchy representation aren't frames,
; but will be used to compose perceptions which use frames

; Object <- StaticObject
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "StaticObject")
   (ConceptNode "Object")
)

; Object <- Unknown
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "Unknown")
   (ConceptNode "Object")
)

; StaticObject <- Structure
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "Structure")
   (ConceptNode "StaticObject")
)

; Object <- MovableObject
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "MovableObject")
   (ConceptNode "Object")
)

; MovableObject <- Item
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "Item")
   (ConceptNode "MovableAgent")
)

; MovableObject <- Agent
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "Agent")
   (ConceptNode "MovableAgent")
)

; Agent <- Humanoid
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "Humanoid")
   (ConceptNode "Agent")
)

; Agent <- Pet
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "Pet")
   (ConceptNode "Agent")
)

; Agent <- Avatar
(InheritanceLink (stv 1.0 1.0) (cog-new-av 0 1 0) 
   (ConceptNode "Avatar")
   (ConceptNode "Agent")
)
