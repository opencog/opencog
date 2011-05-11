; opencog/embodiment/scm/command-resolution-rules.scm
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

; This file contains all the implication links used in the command resolution
; process. If you want to add a new implication link, you must to define it here
; and then add the variable which points to the link into the list pointed by the
; variable: command-resolution-rules (at the end of this file)

(define evaluate-grab-command
  (BindLink (stv 1 1) (cog-new-av 0 1 0)
   (ListLink
    (TypedVariableLink
     (VariableNode "$agent")
     (ListLink
      (VariableTypeNode "ConceptNode")
      (VariableTypeNode "WordInstanceNode")
      )
     )
    (TypedVariableLink
     (VariableNode "$framePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameAgentPredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameDepictivePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameEntityPredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$targetEntity")
     (VariableTypeNode "WordInstanceNode")
     )
    (TypedVariableLink
     (VariableNode "$targetEntitySemeNode")
     (VariableTypeNode "SemeNode")
     )
    (TypedVariableLink
     (VariableNode "$targetRealNode")
     (ListLink      
      (VariableTypeNode "AccessoryNode")
      (VariableTypeNode "ObjectNode")
      )
     )
    )
   (ImplicationLink
    (AndLink
     (InheritanceLink
      (VariableNode "$framePredicateNode")
      (DefinedFrameNode "#Manipulation")
      )
     (InheritanceLink
      (VariableNode "$frameAgentPredicateNode")
      (DefinedFrameElementNode "#Manipulation:Agent")
      )
     (InheritanceLink
      (VariableNode "$frameDepictivePredicateNode")
      (DefinedFrameElementNode "#Manipulation:Depictive")
      )
     (InheritanceLink
      (VariableNode "$frameEntityPredicateNode")
      (DefinedFrameElementNode "#Manipulation:Entity")
      )
       
     ; filter by the given predicateNode
     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameEntityPredicateNode")
      )             
     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameAgentPredicateNode")
      )             
     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameDepictivePredicateNode")
      )
     
     (EvaluationLink
      (VariableNode "$frameAgentPredicateNode")
      (VariableNode "$agent")
      )
     (EvaluationLink
      (VariableNode "$frameDepictivePredicateNode")
      (ConceptNode "#grab")
      )      
     (EvaluationLink
      (VariableNode "$frameEntityPredicateNode")
      (VariableNode "$targetEntity")
      )                 

     (ReferenceLink
      (VariableNode "$targetEntitySemeNode")
      (VariableNode "$targetEntity")
      )
     
     (ReferenceLink
      (VariableNode "$targetRealNode")
      (VariableNode "$targetEntitySemeNode")
      )
         
     )
    
    (ExecutionLink
     (GroundedSchemaNode "scm:createActionCommand")
     (ListLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$agent")
      (GroundedSchemaNode "goto_object_and_grabit")
      (ListLink
       (VariableNode "$targetRealNode")
       )
      )
     )   

           
    ) ; ImplicationLink
   ) ; BindLink
  )

(define evaluate-drop-command
  (BindLink (stv 1 1) (cog-new-av 0 1 0)
   (ListLink
    (TypedVariableLink
     (VariableNode "$source")
     (ListLink
      (VariableTypeNode "ConceptNode")
      (VariableTypeNode "WordInstanceNode")
      )
     )
    (TypedVariableLink
     (VariableNode "$framePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameSourcePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameDepictivePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameThemePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$targetTheme")
     (VariableTypeNode "WordInstanceNode")
     )
    (TypedVariableLink
     (VariableNode "$targetThemeSemeNode")
     (VariableTypeNode "SemeNode")
     )
    (TypedVariableLink
     (VariableNode "$targetRealNode")
     (ListLink
      (VariableTypeNode "AccessoryNode")
      (VariableTypeNode "ObjectNode")
      )
     )
    )
   (ImplicationLink
    (AndLink
     (InheritanceLink
      (VariableNode "$framePredicateNode")
      (DefinedFrameNode "#Motion_directional")
      )
     (InheritanceLink
      (VariableNode "$frameSourcePredicateNode")
      (DefinedFrameElementNode "#Motion_directional:Source")
      )
     (InheritanceLink
      (VariableNode "$frameDepictivePredicateNode")
      (DefinedFrameElementNode "#Motion_directional:Depictive")
      )
     (InheritanceLink
      (VariableNode "$frameThemePredicateNode")
      (DefinedFrameElementNode "#Motion_directional:Theme")
      )
       
     ; filter by the given predicateNode
     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameSourcePredicateNode")
      )
     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameDepictivePredicateNode")
      )
     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameThemePredicateNode")
      )
 
     
     (EvaluationLink
      (VariableNode "$frameSourcePredicateNode")
      (VariableNode "$source")
      )
     (EvaluationLink
      (VariableNode "$frameDepictivePredicateNode")
      (ConceptNode "#drop")
      )
     (EvaluationLink
      (VariableNode "$frameThemePredicateNode")
      (VariableNode "$targetTheme")
      )

     (ReferenceLink
      (VariableNode "$targetThemeSemeNode")
      (VariableNode "$targetTheme")
      )
    
     (ReferenceLink
      (VariableNode "$targetRealNode")
      (VariableNode "$targetThemeSemeNode")
      )
              
     )
    
    (ExecutionLink
     (GroundedSchemaNode "scm:createActionCommand")
     (ListLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$source")      
      (GroundedSchemaNode "drop")
      (ListLink)
      )
     )
           
    ) ; ImplicationLink
   ) ; BindLink
  )

(define evaluate-goto-command
  (BindLink (stv 1 1) (cog-new-av 0 1 0)
   (ListLink
    (TypedVariableLink
     (VariableNode "$agent")
     (ListLink
      (VariableTypeNode "ConceptNode")
      (VariableTypeNode "WordInstanceNode")
      )
     )
    (TypedVariableLink
     (VariableNode "$framePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameThemePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameGoalPredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$goalValue")
     (VariableTypeNode "WordInstanceNode")
     )
    (TypedVariableLink
     (VariableNode "$goalSemeNode")
     (VariableTypeNode "SemeNode")
     )
    (TypedVariableLink
     (VariableNode "$goalRealNode")
     (ListLink
      (VariableTypeNode "AccessoryNode")
      (VariableTypeNode "ObjectNode")
      (VariableTypeNode "AvatarNode")
      (VariableTypeNode "PetNode")
      (VariableTypeNode "StructureNode")
      (VariableTypeNode "HumanoidNode")
      )
     )
    )
   (ImplicationLink
    (AndLink
     (InheritanceLink
      (VariableNode "$framePredicateNode")
      (DefinedFrameNode "#Motion")
      )
     (InheritanceLink
      (VariableNode "$frameThemePredicateNode")
      (DefinedFrameElementNode "#Motion:Theme")
      )
     (InheritanceLink
      (VariableNode "$frameGoalPredicateNode")
      (DefinedFrameElementNode "#Motion:Goal")
      )

     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameThemePredicateNode")
      )
     (FrameElementLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameGoalPredicateNode")
      )

     (EvaluationLink
      (VariableNode "$frameThemePredicateNode")
      (VariableNode "$agent")
      )
     (EvaluationLink
      (VariableNode "$frameGoalPredicateNode")
      (VariableNode "$goalValue")      
      )

     (ReferenceLink
      (VariableNode "$goalSemeNode")
      (VariableNode "$goalValue")
      )     
     
     (ReferenceLink
      (VariableNode "$goalRealNode")
      (VariableNode "$goalSemeNode")
      )

     )
    (ExecutionLink
     (GroundedSchemaNode "scm:createActionCommand")
     (ListLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$agent")
      (GroundedSchemaNode "walkto_obj")
      (ListLink
       (VariableNode "$goalRealNode")
       )
      )
     )

    )
   )
  )

(define command-resolution-rules 
  (list 
   evaluate-grab-command 
   evaluate-drop-command
   evaluate-goto-command
   )
  )
