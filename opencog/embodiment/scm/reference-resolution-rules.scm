; opencog/embodiment/scm/reference-resolution-rules.scm
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

; This file contains all the implication links used in the reference resolution
; process. If you want to add a new implication link, you must to define it here
; and then add the variable which points to the link into the list pointed by the
; variable: command-resolution-rules (at the end of this file)

(define evaluate-color-reference
    (BindLink (stv 1 1) (cog-new-av 1 1 1)
        (ListLink
            (TypedVariableLink
                (VariableNode "$entityValue")
                (VariableTypeNode "WordInstanceNode")
            )

            (TypedVariableLink
                (VariableNode "$colorValue")
                (VariableTypeNode "WordInstanceNode")
            )

            (TypedVariableLink
                (VariableNode "$entityWordNode")
                (VariableTypeNode "WordNode")
            ) 

            (TypedVariableLink
                (VariableNode "$colorWordNode")
                (VariableTypeNode "WordNode")
            )
    
            (TypedVariableLink
                (VariableNode "$framePredicateNode")
                (VariableTypeNode "PredicateNode")
            )

            (TypedVariableLink
                (VariableNode "$frameEntityPredicateNode")
                (VariableTypeNode "PredicateNode")
            )

            (TypedVariableLink
                (VariableNode "$frameColorPredicateNode")
                (VariableTypeNode "PredicateNode")
            )
        ); ListLink

        (ImplicationLink
            (AndLink     
                ; filter by Color frame
                (InheritanceLink
                    (VariableNode "$framePredicateNode")
                    (DefinedFrameNode "#Color")
                )

                (InheritanceLink
                    (VariableNode "$frameEntityPredicateNode")
                    (DefinedFrameElementNode "#Color:Entity")
                )

                (InheritanceLink
                    (VariableNode "$frameColorPredicateNode")
                    (DefinedFrameElementNode "#Color:Color")
                )
      
                (FrameElementLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$frameEntityPredicateNode")
                )

                (FrameElementLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$frameColorPredicateNode")
                )
      
                (EvaluationLink
                    (VariableNode "$frameEntityPredicateNode")
                    (VariableNode "$entityValue")
                )

                (EvaluationLink
                    (VariableNode "$frameColorPredicateNode")
                    (VariableNode "$colorValue")
                )
  
                ; get the WordNodes from the frame elements
                (ReferenceLink
                    (VariableNode "$entityValue")
                    (VariableNode "$entityWordNode")
                )

                (ReferenceLink
                    (VariableNode "$colorValue")
                    (VariableNode "$colorWordNode")
                )
             
            ); AndLink

            ; ImplicationLink output
            (ExecutionLink      
                (GroundedSchemaNode "scm:filterByColor")
                (ListLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$entityValue")
                    (VariableNode "$entityWordNode")
                    (VariableNode "$colorWordNode")
                )
            )
        ); ImplicationLink
    ); BindLink
)

(define evaluate-dimension-reference
    (BindLink (stv 1 1) (cog-new-av 1 1 1)
        (ListLink
            (TypedVariableLink
                (VariableNode "$framePredicateNode")
                (VariableTypeNode "PredicateNode")
            )

            (TypedVariableLink
                (VariableNode "$frameObjectPredicateNode")
                (VariableTypeNode "PredicateNode")
            )

            (TypedVariableLink
                (VariableNode "$frameMeasurementPredicateNode")
                (VariableTypeNode "PredicateNode")
            )

            (TypedVariableLink
                (VariableNode "$objectValue")
                (VariableTypeNode "WordInstanceNode")
            )

            (TypedVariableLink
                (VariableNode "$dimensionValue")
                (VariableTypeNode "WordInstanceNode")
            )

            (TypedVariableLink
                (VariableNode "$dimensionWordNode")
                (VariableTypeNode "WordNode")
            )

            (TypedVariableLink
                (VariableNode "$objectWordNode")
                (VariableTypeNode "WordNode")
            )
        ); ListLink
   
        (ImplicationLink
            (AndLink
                (InheritanceLink
                    (VariableNode "$framePredicateNode")
                    (DefinedFrameNode "#Dimension")
                )
   
                (InheritanceLink
                    (VariableNode "$frameObjectPredicateNode")
                    (DefinedFrameElementNode "#Dimension:Object")
                )
   
                (InheritanceLink
                    (VariableNode "$frameMeasurementPredicateNode")
                    (DefinedFrameElementNode "#Dimension:Measurement")
                )
         
                (FrameElementLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$frameObjectPredicateNode")
                )
   
                (FrameElementLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$frameMeasurementPredicateNode")
                )
         
                (EvaluationLink
                    (VariableNode "$frameObjectPredicateNode")
                    (VariableNode "$objectValue")
                )
   
                (EvaluationLink
                    (VariableNode "$frameMeasurementPredicateNode")
                    (VariableNode "$dimensionValue")
                )             
         
                (ReferenceLink
                    (VariableNode "$dimensionValue")
                    (VariableNode "$dimensionWordNode")
                )      
         
                (ReferenceLink
                    (VariableNode "$objectValue")
                    (VariableNode "$objectWordNode")
                )
            ); AndLink
        
            (ExecutionLink
                (GroundedSchemaNode "scm:filterByDimension")
                (ListLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$objectValue")
                    (VariableNode "$objectWordNode")
                    (VariableNode "$dimensionWordNode")
                )
            )
               
        ); ImplicationLink
    )
)

(define evaluate-distance-reference
    (BindLink (stv 1 1) (cog-new-av 1 1 1)
        (ListLink
            (TypedVariableLink
                (VariableNode "$framePredicateNode")
                (VariableTypeNode "PredicateNode")
            )

            (TypedVariableLink
                (VariableNode "$frameFigurePredicateNode")
                (VariableTypeNode "PredicateNode")
            )

            (TypedVariableLink
                (VariableNode "$frameGroundPredicateNode")
                (VariableTypeNode "PredicateNode")
            )
  
            (TypedVariableLink
                (VariableNode "$frameRelationTypePredicateNode")
                (VariableTypeNode "PredicateNode")
            )
  
            (TypedVariableLink
                (VariableNode "$figureValue")
                (VariableTypeNode "WordInstanceNode")
            )
  
            (TypedVariableLink
                (VariableNode "$groundValue")
                (VariableTypeNode "WordInstanceNode")
            )
  
            (TypedVariableLink
                (VariableNode "$figureWordNode")
                (VariableTypeNode "WordNode")
            )
  
            (TypedVariableLink
                (VariableNode "$groundWordNode")
                (VariableTypeNode "WordNode")
            )
  
            (TypedVariableLink
                (VariableNode "$relationTypeValue")
                (VariableTypeNode "ConceptNode")
            )
  
            (TypedVariableLink
                (VariableNode "$relationTypePredicateNode")
                (VariableTypeNode "PredicateNode")
            )
        ); ListLink

        (ImplicationLink
            (AndLink
                (InheritanceLink
                    (VariableNode "$framePredicateNode")
                     (DefinedFrameNode "#Locative_relation")
                )

                (InheritanceLink
                    (VariableNode "$frameFigurePredicateNode")
                    (DefinedFrameElementNode "#Locative_relation:Figure") 
                )

                (InheritanceLink
                    (VariableNode "$frameGroundPredicateNode")
                    (DefinedFrameElementNode "#Locative_relation:Ground") 
                )

                (InheritanceLink
                    (VariableNode "$frameRelationTypePredicateNode")
                    (DefinedFrameElementNode "#Locative_relation:Relation_type") 
                )

                (FrameElementLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$frameFigurePredicateNode")
                )             

                (FrameElementLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$frameGroundPredicateNode")
                )             

                (FrameElementLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$frameRelationTypePredicateNode")
                )             

                (EvaluationLink
                    (VariableNode "$frameFigurePredicateNode")
                    (VariableNode "$figureValue")
                )

                (EvaluationLink
                    (VariableNode "$frameGroundPredicateNode")
                    (VariableNode "$groundValue")
                )

                (EvaluationLink
                    (VariableNode "$frameRelationTypePredicateNode")
                    (VariableNode "$relationTypeValue") 
                )

                (ReferenceLink
                    (VariableNode "$figureValue") 
                    (VariableNode "$figureWordNode") 
                )

                (ReferenceLink
                    (VariableNode "$groundValue") 
                    (VariableNode "$groundWordNode") 
                )
            ); AndLink

            (ExecutionLink
                (GroundedSchemaNode "scm:filterByDistance")
                (ListLink
                    (VariableNode "$framePredicateNode")
                    (VariableNode "$figureValue")
                    (VariableNode "$figureWordNode") 
                    (VariableNode "$groundValue")
                    (VariableNode "$groundWordNode")
                    (VariableNode "$relationTypeValue") 
                )
            )

        ); ImplicationLink
    ); BindLink
)

(define reference-resolution-rules
    (list 
        evaluate-color-reference
        evaluate-dimension-reference
        evaluate-distance-reference
    )
)
