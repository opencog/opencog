; opencog/embodiment/scm/language-comprehension.scm
;
; TODO: many of the functions can be repalced by using the pattern matcher
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

; This file contains a set of functions used by the process of embodiment disambiguation.
; This process consists in two steps: Reference and Command resolution
; Reference resolution is executed first.


;;; Functions executed by GroundedSchemaNodes (in the effect side ImplicationLinks), used to filter results

; This function retrieves all the semeNodes related to the given WordInstanceNode
; that matches the given dimension using size predicates defined into the application
(define (filterByDimension framePredicateNode wordInstanceNode wordNode dimensionWordNode)
  (let ((tv '())
        (semeNodes '())
        (dimension (cog-name dimensionWordNode))
        )
    (if (inLatestSentence framePredicateNode)
        (begin
          (map
           (lambda (semeNode)
            ; retrieve the is_small EvalLink
             (let ((tv (cog-tv
                        (EvaluationLink
                         (PredicateNode "is_small")
                         (ListLink
                          (get-real-node semeNode)
                          )
                         )
                        )
                       ))
               (if (not (null? tv))
                   (begin
                     ; now check the link TruthValue
                     (if (or (and
                              (> (assoc-ref (cog-tv->alist tv) 'mean) 0)
                              (equal? dimension "small")
                              )
                             (and
                              (= (assoc-ref (cog-tv->alist tv) 'mean) 0)
                              (equal? dimension "large")
                              )
                             )
                         (set! semeNodes (append semeNodes (list semeNode) ) )
                         )                      
                     )
                   )
               )             
             )
           (get-seme-nodes-using-ontology wordNode)
           )
          (ListLink 
           (append (list wordInstanceNode ) semeNodes )     
           )
          )
        '() ; return null if it isn't a predicate of the latest sentence
        )
    )
  )

; This function retrieves all the semeNodes related to the given WordInstanceNode
; that matches the given relationType using distance predicates defined into the application
(define (filterByDistance framePredicateNode figureWIN figureWN groundWIN groundWN relationTypeCN)
  (let ((tv '())
        (predicateNode '())
        (semeNodes '())
        (relation (cog-name relationTypeCN))        
        )
    (cond ((equal? "#near" relation)
           (set! predicateNode (PredicateNode "near"))
           )
          ((equal? "#next" relation)
           (set! predicateNode (PredicateNode "next"))
           )
          )
    (if (and (not (null? predicateNode)) (inLatestSentence framePredicateNode))
        (begin    
          (map
           (lambda (figureSemeNode)
             
             (map
              (lambda (groundSemeNode)
                (let ((tv (cog-tv
                           (EvaluationLink
                            predicateNode
                            (ListLink
                             (get-real-node figureSemeNode)
                             (get-real-node groundSemeNode)
                             )
                            )
                           )
                          ))
                  (if (and (not (null? tv)) (> (assoc-ref (cog-tv->alist tv) 'mean) 0))
                      (set! semeNodes (append semeNodes (list figureSemeNode) ) )
                      )
                  )
                )          
              (get-seme-nodes-using-ontology groundWN)
              )
             )
           (get-seme-nodes-using-ontology figureWN)
           )

          (ListLink 
           (append (list figureWIN ) semeNodes )     
           )
          )
        '() ; return null if it isn't a predicate of the latest sentence
        )
    )
  )

; This function retrieves all the semeNodes related to the given WordInstanceNode
; that matches the given color using color predicates defined into the application
(define (filterByColor framePredicateNode wordInstanceNode wordNode colorWordNode)
  (let ((tv '())
        (semeNodes '())
        (color (cog-name colorWordNode))
        )
    (if (inLatestSentence framePredicateNode)
        (begin
          (map
           (lambda (semeNode)
             (let ((tv (cog-tv
                        (EvaluationLink
                         (PredicateNode "color")
                         (ListLink
                          (get-real-node semeNode)
                          (ConceptNode color)
                          )
                         )
                        )
                       ))
               
               (if (and (not (null? tv)) (> (assoc-ref (cog-tv->alist tv) 'mean) 0))
                   (set! semeNodes (append semeNodes (list semeNode) ) )
                   )               
               )
             )
           (get-seme-nodes-using-ontology wordNode)
           )          
          (ListLink 
           (append (list wordInstanceNode ) semeNodes )
           )
          )
        '() ; return null if it isn't a predicate of the latest sentence
        )
    )
  )

;;; Functions used by GroundedPredicateNodes to filter the results

; This function check if an ReferenceLink connects a given structure to the 
; ConceptNode #you or the agent SemeNode, determining if the given message was
; sent or not to the agent
;
(define (wasAddressedToMe messageTarget)
    (or (equal? messageTarget (ConceptNode "#you"))
        (cog-link 'ReferenceLink messageTarget agentSemeNod)
    )
)

; This function check if a given PredicateNode belongs to the most recent
; parsed sentence
(define (inLatestSentence predicateNode)
    (if (member predicateNode (get-latest-frame-instances) )
        #t
        #f
    )
)

(define (createActionCommand framePredicateNode agentNode actionNode arguments)
    (if (and (inLatestSentence framePredicateNode) 
             (wasAddressedToMe agentNode)
        )

        (ExecutionLink (stv 1 1)
            actionNode
            arguments
        )

        '()
    )
)

;;; Helper functions

; Call some method to prepare instance of frames given an ungrounded predicateNode
;
; The given Frame instance can contains VariableNodes in its elements values
;
; @return true if there is a mapped function that will preprocess the frame, 
;         false otherwise
;
(define (frame-preprocessor frame_instance)
    (let ( (frameType (get-frame-instance-type frame_instance))
         )
         (cond 
             ( (equal? frameType "#Locative_relation")
               ; first remove all old known spatial relations
               (map
                   (lambda (evalLink)              
                       (map 
                           (lambda (oldFrame)
                               (remove-frame-instance oldFrame)
                           )
                           (cog-outgoing-set (car (gdr evalLink)))
                       ); map
                   ); lambda

                   (cog-filter-incoming 'EvaluationLink
                                         (PredicateNode "knownSpatialRelations")
                   )
               ); map

               ; then compute the new spatial relations and re-define the predicate
               (EvaluationLink (stv 1 1)
                   (PredicateNode "knownSpatialRelations" )
                   ; cog-emb-compute-spatial-relations is a scheme binding of 
                   ; LanguageComprehension::execute(objectObserver, 
                   ;                                figureSemeNode, 
                   ;                                groundSemeNode,
                   ;                                ground2SemeNode
                   ;                               )
                   (cog-emb-compute-spatial-relations 
                       (get-sentence-author (car (get-new-parsed-sentences)))
                       (get-grounded-element-value
                           (get-frame-element-instance-value 
                               (get-frame-element-instance frame_instance "Figure")
                           )
                        )
                        (get-grounded-element-value 
                            (get-frame-element-instance-value
                                (get-frame-element-instance frame_instance "Ground")
                            )
                        )
                        ; TODO: what is Ground_2, I can not find it in 
                        ;       #Locative_relation frame
                        (get-grounded-element-value
                            (get-frame-element-instance-value
                                (get-frame-element-instance frame_instance "Ground_2")
                            )
                        )
                   ); cog-emb-compute-spatial-relations
               ); EvaluationLink

               #t
           )
         ); cond

         #f
    ); let
)

(define (remove-frame-instance predicateNode)  
  (let ((removed? #f))
    (if (and (not (null? predicateNode)) (equal? (cog-type predicateNode) 'PredicateNode))
        (begin         
    
          (map
           (lambda (elementLink)
             ; first remove all eval link that connects the 
             ; value to its respetive element
             (let ((elementPredicate (car (gdr elementLink))))
               (map
                (lambda (evalLink)
                  (cog-delete evalLink)
                  )
                (cog-filter-incoming
                 'EvaluationLink
                 elementPredicate
                 )
                )
               
               (map
                (lambda (inheritanceLink)
                  (cog-delete inheritanceLink)
                  )
                (cog-filter-incoming
                 'InheritanceLink
                 elementPredicate
                 )
                )               
               )
             ; disconnect the frame element from the frame instance
             (cog-delete elementLink)
             )
           (cog-filter-incoming
            'FrameElementLink
            predicateNode
            )
           )
         ; finally remove the inheritance link and the instance node
          (map
           (lambda (inheritance)
             (cog-delete inheritance)
             (set! removed? #t)
             )
           (cog-filter-incoming
            'InheritanceLink
            predicateNode
            )
           )          
        )
        )
    removed?
    )
  )

(define (remove-frame-instances frameType)
  (map
   (lambda (inheritance)
     (remove-frame-instance (gar inheritance))
     )
   (cog-get-link
    'InheritanceLink
    'PredicateNode
    (DefinedFrameNode frameType)
    )
   )    
  )

; Retrieve a list containing the sentences that belong to the most
; recent parsed text
;
; TODO: remove 'get-latest-sentences' and use 'get-new-parsed-sentences' directly
(define (get-latest-sentences)
    (get-new-parsed-sentences)
)

; Retrieve the node of the agent who says the given sentence
; 
; @note PAI::processInstruction is responsible for connecting author to the 
;       SentenceNode. And there is an example of how the author is represented 
;       in AtomSpace.
;
; (ListLink (stv 1 1)
;     (AvatarNode "id_4732" (av 0 1 0))
;     (SentenceNode "sentence@00c462c6-e7b1-42f2-bb2d-0d809f49f01c")
; )
;
; @todo We may need a more generic way of labeling SentenceNode, if we want more 
;       infomation rather than just author. 
;
(define (get-sentence-author sentence)
    (let ( (author '())
         )
         (map
             (lambda (link)
                 (if (and (equal? (car (gdr link)) sentence)
                          (cog-subtype? 'ObjectNode (cog-type (gar link)))
                     )
                     (set! author (gar link))
                 )
             ); lambda

             (cog-filter-incoming 'ListLink sentence)
         ); map

         author
    ); let
)

; Retrieve a list containing the parses (ParseNode) that belongs to the most
; recent parsed sentences (SentenceNode)
;
; (ParseLink
;     (ParseNode "sentence@2d19c7e7-2e02-4d5e-9cbe-6772174f3f4d_parse_0")
;     (SentenceNode "sentence@2d19c7e7-2e02-4d5e-9cbe-6772174f3f4d")
; )
;
(define (get-latest-parses)
    (let ( (parse_node_list (list)) )
         (map
             (lambda (sentence)
                 (map 
                     (lambda (parse_link)          
                         (set! parse_node_list
                               (append parse_node_list (list (gar parse_link)) ) 
                         )
                     )
                     ; Get ParseLink
                     (cog-get-link 'ParseLink 'ParseNode sentence)
                 )
             ); lambda

             ; Get a list of SenseNodes with (AnchorNode "# New Parsed Sentence")
             (get-new-parsed-sentences)
         ); map

         ; return value
         parse_node_list
    )
)

; Retrieve a list containing the WordInstanceNodes that belongs to the most
; recent sentences parses
;
; (ReferenceLink (stv 1.0 1.0)
;     (ParseNode "sentence@2d19c7e7-2e02-4d5e-9cbe-6772174f3f4d_parse_0")
;     (ListLink
;         (WordInstanceNode "humans@52c30b4d-5717-47cb-822d-b2caa44f94b9")
;         (WordInstanceNode "have@0f223d17-31a6-49fe-9d37-350d50c53926")
;         (WordInstanceNode "two@f6c2a2a2-232b-4e33-9c93-72f211b475d3")
;         (WordInstanceNode "feet@bf71826c-487e-42df-a941-0ecd3c942a76")
;     )
; )
;
(define (get-latest-word-instance-nodes . parses)
    (let ( (word_instance_node_list (list)) )
         (map
             (lambda (parse_node)
                 (map
                     (lambda (reference_link)
                         (set! word_instance_node_list
                             (append word_instance_node_list 
                                     (cog-outgoing-set (car (gdr reference_link)) ) 
                             ) 
                         )
                     )
                    
                     ; Get a list of ReferenceLink containing parse_node
                     (cog-get-link 'ReferenceLink 'ListLink parse_node)
                 )
             )

             (if (= (length parses) 0) 
                 (get-latest-parses) 
                 (if (list? (car parses))
                     (car parses)
                     parses
                 ); if
             ); if     
         ); map

         ; return value
         word_instance_node_list
    ); let
)

; Return a list of Frames (PredicateNodes) which belong to the most recent 
; parsed sentences.
;
; It uses the most recent WordInstanceNodes to find the PredicateNodes.
;
; @note Here is an example of how the current RelEx2Frame output stores a Frame Element, 
;
;       Sentence: The ball is red.
;
;       InheritanceLink (stv 1 1)
;           PredicateNode "red@87cd46cc-006d-4ff4-b4de-7e579f92adf6_Color_Entity"
;           DefinedFrameElementNode "#Color:Entity"
;       
;       FrameElementLink (stv 1 1)
;           PredicateNode "red@87cd46cc-006d-4ff4-b4de-7e579f92adf6_Color"
;           PredicateNode "red@87cd46cc-006d-4ff4-b4de-7e579f92adf6_Color_Entity"
;           
;       EvaluationLink (stv 1 1)
;           PredicateNode "red@87cd46cc-006d-4ff4-b4de-7e579f92adf6_Color_Entity"
;           WordInstanceNode "ball@72673eaa-74db-4ff1-9ce9-f57f8805d20c"
;
(define (get-latest-frame-instances . wordInstanceNodes)
    (let ( (frame_instances (list)) 
         )
         (map
             (lambda (win)
                 (map
                     (lambda (evaluation_link)
                         (let ( (frame_element_instance (gar evaluation_link) )
                              )
                              (if (not (null? 
                                           (cog-get-link 
                                               'InheritanceLink
                                               'DefinedFrameElementNode
                                                frame_element_instance
                                           )
                                       ); null?
                                  ); not
 
                                  (let ( (frame_element_links 
                                             (cog-get-link
                                                 'FrameElementLink
                                                 'PredicateNode
                                                 frame_element_instance
                                             )
                                         ); frame_element_links
                                       )
 
                                       (if (and (not (null? frame_element_links))
                                                (gar (car frame_element_links))
                                                (not (null?
                                                         (cog-get-link
                                                             'InheritanceLink
                                                             'DefinedFrameNode
                                                             (gar (car frame_element_links))
                                                         )
                                                     ); null?
                                                ); not
                                           ); and
 
                                           (set! frame_instances 
                                               (append frame_instances
                                                   (list (gar (car frame_element_links)))
                                               )
                                           )
                                       ); if
                                  ); let
                              ); if
                         ); let
                     ); lambda

                     ; Get all the EvaluationLinks containing FrameElement
                     ; instances (PredicateNodes)
                     (cog-get-link 'EvaluationLink 'PredicateNode win)
                 ); map
             );lambda 

             ; Get the list of currently parsed WordInstanceNode
             (if (= (length wordInstanceNodes) 0) 
                 (get-latest-word-instance-nodes) 

                 ; TODO: why not just use wordInstanceNodes???
                 (if (list? (car wordInstanceNodes))
                     (car wordInstanceNodes)
                      wordInstanceNodes
                 )
             ); if
         ); map

         ; return a list if Frame instances (PredicateNodes)
         (delete-duplicates frame_instances)
    ); let
)

; Retrieve all the anaphoric suggestions for a given WordInstanceNode
; that belong to the most recent parsed sentence 
;
; @return 
;
; ( (suggestion_win_1 . strength_1)
;   (suggestion_win_2 . strength_2)
;   ...
; )
;
; @note The anaphoric suggestions are stored as follows, 
;
;     EvaluationLink
;        ConceptNode "anaphoric reference"
;        ListLink
;           WordInstanceNode <- pronoun
;           WordInstanceNode <- suggestion
;
(define (get-anaphoric-suggestions pronoun_win)
    (let ( (suggestions '()) 
         )
         (map
             (lambda (suggestion_evaluation_link)
                 (let ( (list_link (car (gdr suggestion_evaluation_link) ) )
                        (strength
                            (get_truth_value_confidence (cog-tv suggestion_evaluation_link) )  
                        )
                      )
                      
                      (if (equal? (gar list_link) pronoun_win)
                          (set! suggestions
                                (append suggestions 
                                       (list (cons (car (gdr list_link)) strength)) 
                                ) 
                          )
                      )
                 )
             ); lambda

             (cog-get-link 'EvaluationLink 'ListLink 
                           (ConceptNode "anaphoric reference") 
             )
         ); map

         ; return value
         suggestions
    ); let
)

; Retrieve the WordNode related to a given WordInstanceNode
;
; @note This is an example below, 
;
;       ReferenceLink
;           WordInstanceNode "red@216e8536-4867-49bc-970a-fc69608e39d2"
;           WordNode "red"
;      
(define (get-word-node wordInstanceNode)
    (let ( (reference_link_list
               (cog-get-link 'ReferenceLink 'WordNode wordInstanceNode)
           )
         )

         (if (not (null? reference_link_list) )
             (car (gdr (car reference_link_list)))
             '()
         )    
    )
)

; Retrieves all the SemeNodes attached to a given WordNode. 
;
; Each WordNode can have many SemeNodes attached to it by a ReferenceLink as below
;
;     ReferenceLink
;         SemeNode
;         WordNode  
;
; TODO: a better solution is using pattern matcher to do this job. 
;
(define word-node-seme-nodes-cache '() )

(define (get-seme-nodes wordNode)
    (fold 
        (lambda (refLink result)
            (append result (list (gar refLink)) )
        )
        '()
        (cog-get-link 'ReferenceLink 'SemeNode wordNode)
   )
)

; Return the corresponding object nodes (or a child of it) of SemeNode. 
;
; Each SemeNode is connected to a node that represents a real object into the 
; environment. The format is as follows: 
;
; ReferenceLink
;    ObjectNode (or a child of it)
;    SemeNode  
;
; TODO: if each SemeNode is connected with an ObjectNode (or child of it), why 
;       not just use the ObjectNode directly???
;
(define (get-real-node semeNode)
    (let ( (realObject '() )
         )
         (map
             (lambda (candidateLink)
                 (let ( (object (cog-get-partner candidateLink semeNode) ) 
                      )
                      (if (cog-subtype? 'ObjectNode (cog-type object) )
                          (set! realObject object)
                      )
                 )
             ); lambda

             ; Get all the ReferenceLink containing the given SemeNode
             (cog-filter 
                 'ReferenceLink 
                 (cog-incoming-set semeNode)
             )
         ); map

         ; return value
         realObject
    )  
)

; Given a WordInstanceNode, this function will try to find all the Corresponding
; SemeNodes.
;
; SemeNodes can be grounded by WordInstanceNodes of type nouns and pronouns, so
; only this two types of WordInstanceNodes will return candidate SemeNodes. 
;
; SemeNodes are connected to WordNodes, so getting the WordNode of the 
; WordInstanceNode it is possible to retrieve its SemeNodes. 
;
; If the WordInstanceNode was a pronoun the anaphoric suggestions will be used
; to build the SemeNodes list. Each WordInstanceNode, suggested in the anaphora,
; will have its WordNode evaluated and, consequently, all the SemeNodes related 
; to these WordNodes will become part of the final list .
;
; The final list contains not only the SemeNodes but its strengths 
;
; i.e. ( (0.03 .  (SemeNode "1") )
;        (0    .  (SemeNode "2") )
;        (1    .  (SemeNode "3") ) 
;      )
;
(define (get-candidates-seme-nodes win)
    (let ( (noun_suggestions '())
           (semeNodes '())
         )

         ; Step 1: ground proun to nouns
         ;
         ;         ( (suggestion_win_1 . strength_1)
         ;           (suggestion_win_2 . strength_2)
         ;           ...
         ;         )
         ;
         (cond 
             ; If it is a pronoun, ground it to suitable nouns firstly 
             ( (not (null? (cog-link 
                           'InheritanceLink
                            win
                            (DefinedLinguisticConceptNode "pronoun")           
                           ) 
                    ) 
                )

                ; look for anaphoric reference
                (let ( (anaphoric_suggestions (get-anaphoric-suggestions win) ) )
                     (if (not (null? anaphoric_suggestions) )
                         (map
                             (lambda (suggested_win)
                                 (set! noun_suggestions
                                       (append noun_suggestions (list suggested_win) ) 
                                 )
                             )
                             anaphoric_suggestions
                         ); map

                         (set! noun_suggestions
                               (append noun_suggestions (list (cons win 0) ) ) 
                         )
                     ); if
                ); let          
             )

             ; If it is a noun, add it to the noun_suggestions
             ( (not (null? (cog-link 
                           'PartOfSpeechLink 
                            win
                            (DefinedLinguisticConceptNode "noun")
                           )
                    )
                )

                (set! noun_suggestions
                      (append noun_suggestions (list (cons win 0) ) )
                )
            )
         ); cond

         ; Step 2: ground nouns to SemeNodes 
         (map
             (lambda (candidate)
                 (let* ( (noun_win (car candidate) )
                         (strength (cdr candidate) )
                         (grounded_reference_link (cog-get-link 'ReferenceLink 'SemeNode noun_win) )
                       )
 
                       (if (not (null? grounded_reference_link))
                           ; If the noun WordInstanceNode has already been 
                           ; grounded to suitable SemeNode, use the result 
                           ; directly. 
                           (set! semeNodes
                                 (append semeNodes 
                                         (list (cons strength (list (gar (car grounded_reference_link)) ) ) )
                                 )
                           )

                           ; If the noun WordInstanceNode has not been grounded 
                           ; to any SemeNode yet, get the corresponding WordNode 
                           ; and then get all the SemeNodes realted to the WordNode
                           (let ( (wordNode (get-word-node noun_win) ) 
                                ) 
                                (if (not (null? wordNode))
                                    (set! semeNodes
                                          (append semeNodes
                                                  (list (cons strength (get-seme-nodes-using-ontology wordNode)) ) 
                                          )
                                    )
                                ) ; if
                           ); let

                       ); if

                 ); let
             ); lambda

             noun_suggestions
         ); map

         ; return value
         semeNodes
    ); let
)

; Remove duplicated elements from given list
; TODO: why not use scheme built in function 'delete-duplicates'???
(define (unique-list ls)
    (if (list? ls)
        (let ( (finalList '())
             )
             (map
                 (lambda (element)
                     (if (not (member element finalList))
                         (set! finalList 
                             (append finalList (list element))
                         )
                     )
                 )
                 ls
             )

             finalList
        ); let

        ls
    ); if
)

; Given a list of candidates (objects) in the format:
; ( (WordInstanceNode1 SemeNode1, ..., SemeNodeN)
;   (WordInstanceNode2 SemeNode1, ..., SemeNodeN)
;   (WordInstanceNodeM SemeNode1, ..., SemeNodeN) 
; )
;
; and a list of strengths:
; ( (SemeNode1 0)
;   (SemeNode2 0.3)
;   (SemeNodeM 1.0)
; )
;
; This function tries to keep just on WordInstanceNode and a corresponding SemeNode.
; If there is one SemeNode which has a greater strength than others it will happen.
; However, a list containing the SemeNodes with greater strengths for each
; WordInstanceNode will be returned
;
(define (filter-by-strength objects strengths)
    (let ( (filtered_objects '() ) 
         )
         (map
             (lambda (object)
                 (let ( (win_key (car object) )
                        (seme_node_values (cdr object) )
                        (selected_seme_nodes '() )
                        (strongest 0) ; initial threshold
                      )

                      ; Pick up SemeNodes with strongest strength. There might 
                      ; be more than one SemeNode share the same strongest
                      ; strength. 
                      (map
                          (lambda (semeNode)
                              (cond
                                  ( (> (assoc-ref strengths semeNode) strongest)
                                    (set! selected_seme_nodes (list semeNode) )
                                    (set! strongest (assoc-ref strengths semeNode) )
                                  )
       
                                  ( (= (assoc-ref strengths semeNode) strongest)
                                    (set! selected_seme_nodes 
                                        (append selected_seme_nodes (list semeNode) )
                                    )
                                  )
                              ); cond
                          ); lambda
                          seme_node_values
                      ); map

                      (if (null? selected_seme_nodes)
                          ; If found none SemeNodes, i.e. all the strength of
                          ; SemeNodes are less than initial threshold 0, use 
                          ; all the old SemeNodes, because we don't know which
                          ; ones are better
                          (set! filtered_objects
                              (append filtered_objects object) 
                          )

                          ; If found SemeNodes with strongest strength, add object 
                          ; with selected SemeNodes to filtered object list
                          (set! filtered_objects 
                              (append filtered_objects 
                                      (list (cons win_key selected_seme_nodes) ) 
                              ) 
                          )
                      ); if

                 ); let
             ); lambda

             objects
         );map

         ; return filtered objects, i.e. a list of (WordInstanceNode . SemeNodes)
         filtered_objects
    ); let
)

; Return a new list of the objects, which contains only the SemeNodes listed in
; the SemeNodes list.
;
; Given a list of objects to be filtered in the format:
; ( (WordInstanceNode1 SemeNode1, ..., SemeNodeN)
;   (WordInstanceNode2 SemeNode1, ..., SemeNodeN)
;   (WordInstanceNode2 SemeNode1, ..., SemeNodeN)
; )
;
; and a list of SemeNodes that will be used
; to filter the objects list.
;
(define (filter-objects objects win_and_seme_nodes)
    (let* ( (win_key (car win_and_seme_nodes) )
            (seme_node_values (cdr win_and_seme_nodes) )
            (new_seme_node_values '() )
            (filterObjects (assoc-ref objects win_key) )
          )

          ; Get SemeNodes in both objects and win_and_seme_nodes
          (if filterObjects
              (set! new_seme_node_values 
                    (filter (lambda (x) (member x seme_node_values))
                            filterObjects
                    )
              )
          )

          ; Return updated objects i.e. a list of (WordInstanceNode. SemeNodes)
          (if (null? new_seme_node_values)
              ; If new_seme_node_values is empty, delete corresponding object 
              (delete (assoc win_key objects) objects)

              ; If new_seme_node_values is not empty, delete old object and 
              ; append new one
              (append (delete (assoc win_key objects) objects) 
                      (list (cons win_key new_seme_node_values) )
              )
          )

    ); let*
)

; Return the nearest SemeNode to the current agent (pet), given a list of 
; SemeNodes.
;
; To determine the distance between the agent and the SemeNode (which represents 
; a real element into the environment), the TruthValue of the predicate "proximity"
; is evaluated. The higher truthvalue indicates lower distance. 
;
(define (get-nearest-candidate semeNodes)
    (let ( (agentNode (get-real-node agentSemeNode) )
           (candidates '() ) ; a list of (SemeNode . proximity)
         )   
         (map
             (lambda (candidate_seme_node)
                 (let ( (proximity 
                        (assoc-ref 
                            (cog-tv->alist 
                                (cog-tv 
                                    (EvaluationLink
                                        (PredicateNode "proximity")
                                        (ListLink
                                            agentNode
                                            (get-real-node candidate_seme_node)
                                        )
                                    )
                                )
                            ) 
                            'mean 
                        )
                        ); proximity
                      )

                      (if (or (= (length candidates) 0) 
                              (> proximity (cdr (car candidates)))
                          )
                          (set! candidates (list (cons candidate_seme_node proximity)))

                          (if (and (> (length candidates) 0) 
                                   (= proximity  (cdr (car candidates)))
                              )
                              (set! candidates
                                  (append candidates 
                                      (list (cons candidate_seme_node proximity))
                                  )
                              )
                          )
                      )
                 ); let       
             ); lambda

             semeNodes
         ); map

         (if (> (length candidates) 0)
             (car (car candidates))
             '()
         )
    ); let
)

;;;  Frames manipulation helper functions
; The following example of a Frame/Frame instance will be
; used to explain the functionalities of the helper functions
; described bellow
; Frame: 
; Color:Entity = ball_99
; Color:Color = blue
;
; Definition: 
; (FrameElementLink
;   (DefinedFrameNode "#Color")
;   (DefinedFrameElementNode "#Color:Entity")
; )
; (FrameElementLink
;   (DefinedFrameNode "#Color")
;   (DefinedFrameElementNode "#Color:Color")
; )
;
; Instantiation:
; (InheritanceLink
;   (PredicateNode "ball_99_color")
;   (DefinedFrameNode "#Color")
; )
; (InheritanceLink
;   (PredicateNode "ball_99_color_Entity")
;   (DefinedFrameElementNode "#Color:Entity")
; )
; (InheritanceLink
;   (PredicateNode "ball_99_color_Color")
;   (DefinedFrameElementNode "#Color:Color")
; )
; (FrameElementLink
;   (PredicateNode "ball_99_color")
;   (PredicateNode "ball_99_color_Entity")
; )
; (FrameElementLink
;   (PredicateNode "ball_99_color")
;   (PredicateNode "ball_99_color_Color")
; )
; (EvaluationLink
;   (PredicateNode "ball_99_color_Entity")
;   (SemeNode "ball_99")
; )
; (EvaluationLink
;   (PredicateNode "ball_99_color_Color")
;   (ConceptNode "blue")
; )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Helper functions dealing with frames
;
; @see http://wiki.opencog.org/w/RelEx2Atoms
;      http://wiki.opencog.org/w/Frame2PLN
;      http://framenet.icsi.berkeley.edu/fnReports/data/frameIndex.xml?banner=/fnReports/banner.html
;
; @note below is an example of "red ball"
;
;    ; red@xxx_Color_Entity is an instance of FrameElement #Color:Entity
;    (InheritanceLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;        (DefinedFrameElementNode "#Color:Entity")
;    )
;
;    ; red@xxx_Color_Entity belongs to Frame instance red@xxx_Color
;    (FrameElementLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;    )
;
;    ; word ball@xxx stands for the FrameElement instance red@xxx_Color_Entity
;    (EvaluationLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;        (WordInstanceNode "ball@8631fad6-f29d-4b15-905c-8594fa1d27d3")
;    )
;
;    ; red@xxx_Color_Color is an instance of FrameElement #Color:Color
;    (InheritanceLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
;        (DefinedFrameElementNode "#Color:Color")
;    )
;
;    ; red@xxx_Color_Color belongs to Frame instance red@xxx_Color
;    (FrameElementLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
;    )
;
;    ; word red@xxx stands for the FrameElement instance red@xxx_Color_Color
;    (EvaluationLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
;        (WordInstanceNode "red@701fe254-80e7-4329-80b4-8f865b665843")
;    )
;
;    ; red@xxx_Color is an instance of Frame #Color
;    (InheritanceLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;        (DefinedFrameNode "#Color")
;    )
;
;    The format above is unnecessarily complex, Jared proposed a much more neat 
;    and easily understand format as follows, 
;    
;    (EvaluationLink 
;        (VariableNode "$frameElement")
;        (ListLink
;            (VariableNode "$frameInstance")
;            (VariableNode "$value")
;        )
;    )
;
;    Then the same example can be greatly simplified to: 
;
;    ; word ball@xxx stands for the FrameElement instance red@xxx_Color_Entity, 
;    ; which is an intance of FrameElement #Color:Entity
;    (EvaluationLink (stv 1 1)
;        (DefinedFrameElementNode "#Color:Entity")
;        (ListLink
;            (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;            (WordInstanceNode "ball@8631fad6-f29d-4b15-905c-8594fa1d27d3")
;        )    
;    )
;
;    ; word red@xxx stands for the FrameElement instance red@xxx_Color_Color, 
;    ; which is an instance of FrameElement #Color:Color
;    (EvaluationLink (stv 1 1)
;        (DefinedFrameElementNode "#Color:Color")
;        (ListLink
;            (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
;            (WordInstanceNode "red@701fe254-80e7-4329-80b4-8f865b665843")
;        )
;    )
;
;    We tend to used the new format, while it involves a lot of work to modify
;    current nlp pipeline, such as reference resolution. 
;

; Return FrameElement type given a FrameElement instance (PredicateNode)
; 
; @note For example, if AtomSpace contains the info below, 
;
;    (InheritanceLink (stv 1 1)
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;        (DefinedFrameElementNode "#Color:Entity")
;    )
; 
;    Then, 
;
;    (get-frame-element-instance-type
;        (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;    )
;
;    returns #Color:Entity
;
(define (get-frame-element-instance-type frame_element_instance)
    (let ( (links (cog-get-link
                      'InheritanceLink
                      'DefinedFrameElementNode
                      frame_element_instance
                  )
           )
         )
         (if (not (null? links))
             (cog-name (car (gdr (car links)) ))
             '()
         )
    )    
)

; Return Frame type given a Frame instance (PredicateNode)
;
; @note For example, if AtomSpace contains the info below, 
;
;       InheritanceLink (stv 1 1)
;           PredicateNode "red@2e7cbcab-f074-4b8b-b1db-abbf8b359913_Color"
;           DefinedFrameNode "#Color"
;
;       (get-frame-instance-type
;           (PredicateNode "red@2e7cbcab-f074-4b8b-b1db-abbf8b359913_Color") 
;       ) 
;
;       returns "#Color"
;
(define (get-frame-instance-type frame_nstance)
    (let ( (links (cog-get-link
                      'InheritanceLink
                      'DefinedFrameNode
                      frame_instance
                  )
           )
         )

         (if (not (null? links))
             (cog-name (car (gdr (car links)) ))
             '()
         )
    ); let
)

; Return a list of FrameElement instances (PredicateNodes) give a Frame instance
; (PredicateNode). 
;
; @note For example, if the AtomSpace contains those info below
;
;     (FrameElementLink (stv 1 1)
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
;     )
;
;     (FrameElementLink (stv 1 1)
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;     )
;
;     Then, 
;
;     (get-frame-element-instances
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;     )
;
;     returns
;
;     (list
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;     )
;
(define (get-frame-element-instances frame_instance)
    (let ( (frame_element_instances '())
         )
         (map
             (lambda (frame_element_link)
                 (let ( (frame_element_instance (car (gdr frame_element_link)))
                      )
                      ; TODO: if statement here seems unnecessary
;                      (if (and (equal? 'PredicateNode (cog-type frame_element_instance))
;                               (not (equal? frame_instance frame_element_instance))
;                          )
                          (set! frame_element_instances
                              (append frame_element_instances 
                                  (list frame_element_instance)
                              )
                          )
;                      ); if
                 ); let
             ); lambda

             (cog-filter-incoming 'FrameElementLink frame_instance)
         ); map

         ; return value
         frame_element_instances
    ); let
)

; Given a PredicateNode, which representes a Frame element that belongs to a 
; Frame instance, this function returns the Element value. 
;
; @note For example, if the AtomSpace contains the info below, 
; 
;       EvaluationLink (stv 1 1)
;           PredicateNode "red@77e1dcaf-133e-4578-bf7e-5b776375f5bc_Color_Entity"
;           WordInstanceNode "ball@bf585bf2-32f3-461a-92cf-b307971d43fe"
;
;       (get-frame-element-instance-value
;           (PredicateNode "red@77e1dcaf-133e-4578-bf7e-5b776375f5bc_Color_Entity") 
;       ) 
;       
;       returns (WordInstanceNode "ball@bf585bf2-32f3-461a-92cf-b307971d43fe")
;
(define (get-frame-element-instance-value frame_element_instance)
    (if (not (null? frame_element_instance))
        (let ( (evaluation_links
                   (cog-filter-incoming 'EvaluationLink frame_element_instance)
               )
             )

             (if evaluation_links
                 (car (gdr (car evaluation_links)))
                 '()
             )
        ); let

        '()
    ); if
)

; Return corresponding FrameElement instance (PredicateNode), given a Frame instance
; (PredicateNode) and a specific FrameElement name (second part of the element type, 
; after the colon i.e. given #Color:Entity, frame element name is 'Entity')
;
; @note For example, if the AtomSpace contains those info below
;
;     (FrameElementLink (stv 1 1)
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
;     )
;
;     (FrameElementLink (stv 1 1)
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;     )
;
;     Then, 
;
;     (get-frame-element-instance
;         (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color")
;         "Entity"
;     )
;
;     returns
;
;     (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Entity")
;
(define (get-frame-element-instance frame_instance element_name)  
    (let ( (frame_element_instance_with_given_name '()) 
         )
         (map
             (lambda (frame_element_instance)
                 (let* ( (type (get-frame-element-instance-type frame_element_instance) )
                         (colon_index (string-index type #\:) )
                       )

                       ; Pick up the FrameElement instance with given FrameElement name
                       ;
                       ; do a split in the element type and compare it without the frame prefix
                       ; i.e. its type is #Color:Entity, but compares only the second part Entity == element_name
                       (if (and colon_index
                                (string=? (substring type 
                                                     (+ colon_index 1) 
                                                     (string-length type)
                                          ) 
                                          element_name
                                ) 
                           )
                           (set! frame_element_instance_with_given_name frame_element_instance)
                       )
                 ); let*
             ); lambda

             ; Get all the FrameElement instances
             (get-frame-element-instances frame_instance)
         ); map

         ; return value
         frame_element_instance_with_given_name
    ); let
)

; Given the element type of a specific Frame in a string
; format, this function returns the element name of it (string format).
; The element name is the second part of the Frame type, after 
; the colon symbol.
; i.e. (get-frame-element-name 
;        "#Color:Entity" ) = "Entity"
(define (get-frame-element-name elementType)
  (if (not (null? elementType))
      (let ((colonIndex (list-index (lambda (char) (char=? char #\:)) (string->list elementType))))
        (substring elementType (+ colonIndex 1) (string-length elementType))
        )
      '()
      )
  )

; Given a specific Frame type (string format) and an element name (string format),
; this function returns the DefinedFrameElementNode which represents the desired
; FrameElement. 
;
; i.e. (get-frame-element-node "#Color" "Entity") = 
;         (DefinedFrameElementNode "#Color:Entity")
;
;      (get-frame-element-node "#Color" "Sunda") = '() 
;
(define (get-frame-element-node frameType elementName)
    (let ( (chosenElementNode '())
         )
         (map
             (lambda (elementNode)
                 (let* ( (name (cog-name elementNode))
                         (colonIndex (string-index name #\:) )
                       )
                       ; do a split in the element name and compare it without the frame prefix
                       ; i.e. it name is #Color:Entity, but compares only the second part Entity == elementName
                       (if (and colonIndex
                                (string=? (substring name (+ colonIndex 1) (string-length name)) elementName) 
                           )
                           (set! chosenElementNode elementNode)
                       ); if
                 ); let*
             ); lambda

             (get-frame-elements frameType)
         ); map

         ; return value
         chosenElementNode
    ); let
)


; Given the type of a valid Frame, this function will returns a list containing
; all the elements of the Frame, represented by DefinedFrameElementNodes.
;
; i.e. (get-frame-elements "#Color") = 
;          ( (DefinedFrameElementNode "#Color:Entity")
;            (DefinedFrameElementNode "#Color:Color")
;          )
;
(define (get-frame-elements frameType)
    (let ( (elements '())
           (frameNode (DefinedFrameNode frameType))
           (elementNames '())
         )
         (map
             (lambda (link)
                 (let* ( (element (car (gdr link)))
                         (elementName (get-frame-element-name (cog-name element)))
                       )
                       (set! elements (append elements (list element)))
                       (set! elementNames (append elementNames (list elementName)))
                 )
             ); lambda

             (cog-get-link 'FrameElementLink 'DefinedFrameElementNode frameNode)    
         ); map
    
         (map
             (lambda (link)
                 (let ( (parent (car (gdr link)))
                      )
                      (if (and (equal? 'DefinedFrameNode (cog-type parent))
                               (not (equal? parent frameNode)) 
                          )
                          (map
                              (lambda (parentElement)
                                  (let ( (elementName (get-frame-element-name (cog-name parentElement)))
                                       )
                                       (if (not (member elementName elementNames))
                                           (begin
                                               (set! elements (append elements (list parentElement)))
                                               (set! elementNames (append elementNames (list elementName)))
                                           );begin
                                       ); if
                                  ); let
                              ); lambda

                              (get-frame-elements (cog-name parent))
                          ); map
                      ); if
                 ); let
             ); lambda

             (cog-filter-incoming 'InheritanceLink frameNode)
         ); map

         ; return value
         elements
    ); let
)

; Nodes of several types can be part of structures which has a semantic value
; (i.e. a sentence said by an avatar, composed by several frames). However, the 
; agent's perceptions structures, stored into the AtomSpace, uses a few types 
; of nodes to keep the maintenance of the knowledge database as easier as possible.
; So, this function normalizes a node given as argument to make it useful in the
; PatternMatching process. 
;
; i.e. (get-corresponding-node (WordNode "ball") ) = (ConceptNode "ball")
;      (get-corresponding-node (WordNode "hungry") ) = (ConceptNode "hunger")
;
(define (get-corresponding-node word)
    (let ( (node
               (assoc-ref (get-word-node-map) word)
           )
         )
    
         (if node
             node
             (ConceptNode word)
         )
    )
)

; This method returns true (#t) if the given
; node is a PredicateNode which represents an element
; of a Frame instance and false(#f) otherwise
; i.e. (is-frame-instance-element? (PredicateNode "ball_99_color_Entity")) = #t
;      (is-frame-instance-element? (PredicateNode "ball_99_color")) = #f
(define (is-frame-instance-element? predicateNode)
  (not (null? (cog-get-link 'InheritanceLink 'DefinedFrameElementNode predicateNode)))      
)

; This method returns true (#t) if the given
; node is a PredicateNode which represents a frame instance
; and false(#f) otherwise
; i.e. (is-frame-instance? (PredicateNode "ball_99_color_Entity")) = #f
;      (is-frame-instance? (PredicateNode "ball_99_color")) = #t
(define (is-frame-instance? predicateNode)
  (not (null? (cog-get-link 'InheritanceLink 'DefinedFrameNode predicateNode)))
)



; Given a Frame type(string format) and Element type(string format) and
; a Node which represents the Element value, this function returns a
; list of Frame instances of the given type, which its element of the
; given element type is set with the given value
; i.e. (get-frame-instances-given-an-element "#Color" "Color" (ConceptNode "blue")) = (
;           (PredicateNode "ball_99_color")
;         )
(define (get-frame-instances-given-an-element frameType elementType elementValue)
  (let* (
         (instances '())
         (colonIndex (list-index (lambda (char) (char=? char #\:)) (string->list elementType)))
         (elementName (substring elementType (+ colonIndex 1) (string-length elementType)))
         (frameElementNode (get-frame-element-node frameType elementName))
         )

    (if (not (null? frameElementNode))
        (map
         (lambda (evalLink)
           (if (not (null? (cog-link 'InheritanceLink (gar evalLink) frameElementNode)))
               (map
                (lambda (frameLink)
                  (if (not (null? (cog-link 'InheritanceLink (gar frameLink) (DefinedFrameNode frameType))))
                      (set! instances (append instances (list (gar frameLink))))
                      )
                  )
                (cog-get-link 'FrameElementLink 'PredicateNode (gar evalLink))
                )
               )
           )
         (cog-get-link 'EvaluationLink 'PredicateNode elementValue)
         )
        ) ; if
    
    (delete-duplicates instances)
    )
  )


; Given a Frame type(string format) and a specific value,
; this function retrieves all instances of the given Frame, which
; has at least one element which has its value set with the given value
; i.e. (get-frame-instances-given-its-type-and-an-element-value "#Color" (SemeNode "ball_99")) = (
;      (PredicateNode "ball_99_color")
;         )
(define (get-frame-instances-given-its-type-and-an-element-value frameType value)
  (let* ((instances '())
         (elements (get-frame-elements frameType))
         (links (cog-get-link 'EvaluationLink 'PredicateNode value))
         )
    (if (not (null? links))
        (map
         (lambda (definedFrameElement)       
           (map
            (lambda (link)              
              (if (not (null? (cog-link 'InheritanceLink (gar link) definedFrameElement)))
                 ; the value really belongs to a frame instance of the given type
                 ; now get the predicate of the instance                  
                  (set! 
                   instances 
                   (append 
                    instances
        (list
         (gar (car (cog-get-link 'FrameElementLink 'PredicateNode (gar link))))                    
         )
                    )
                   )
                  
                  )
              )
            links
            )
           )
         elements 
         )        
        )
    instances
    )

  )

; This function checks if at least one element of
; an instance of the Given frame type was set with the given
; value.
; i.e. (belongs-to-frame-instance-of-type? "#Color" (SemeNode "ball_99")) = #t
;      (belongs-to-frame-instance-of-type? "#Color" (SemeNode "ball_00")) = #f
;      (belongs-to-frame-instance-of-type? "#Color" (ConceptNode "blue")) = #t
;      (belongs-to-frame-instance-of-type? "#Color" (ConceptNode "red")) = #f
(define (belongs-to-frame-instance-of-type? frameType value)
  (let* (
         (elements (get-frame-elements frameType))
         (belongs? (not (null? elements)))
         (links '())
         )
    (map
     (lambda (element)
       (set! links (cog-get-link 'EvaluationLink 'PredicateNode value))
       (if (null? links)
           (set! belongs? #f)
           )
       (map
        (lambda (link)
          (if (null? (cog-link 'InheritanceLink (gar link) element))
              (set! belongs? #f)
              )
          )
        links        
        )
       
       )
     elements
     )
    belongs?
    )  
  )

; Return true if a given Frame instance (PredicateNode) has VariableNodes in its
; values of FrameElement instances. 
;
; i.e. (frame-instance-contains-variable? (PredicateNode "ball_99_color")) = #f
; 
; A Frame instance can be prepared to be used in a Pattern Matching operation.
; Suppose we know the value of one element of a specific Frame instance which 
; has two elements and we want to know the value of the other element. So, we
; could prepare a Frame instance with a VariableNode in the value of the desired
; element and do a PatternMatching. 
;
(define (frame-instance-contains-variable? frame_instance)
    (let ( (containsVariable? #f)
         )
         (map
             (lambda (frame_element_instance)
                 (let ( (value (get-frame-element-instance-value frame_element_instance))
                      )
                      (if (and (not (null? value))
                               (equal? 'VariableNode (cog-type value))
                          )
                          (set! containsVariable? #t)
                      )
                 )
             ); lambda

             (get-frame-element-instances frame_instance)
         ); map

         ; return value
         containsVariable?
    )
)

; Check if a given Frame instance, represented by the given PredicateNode,
; has a correspondent instance which has SemeNodes and ConceptNodes in its
; elements, what means that there is a "generated by the agent" Frame 
; instance into the AtomSpace
; i.e. (is-frame-instance-grounded? (PredicateNode "ball_99_color")) = #t
(define (is-frame-instance-grounded? predicateNode)
  (not (null? (get-grounded-frame-instance-predicate predicateNode)))
  )

; This function looks for a Frame instance which grounds the Frame
; instance represented by the given PredicateNode
; i.e (get-grounded-frame-instance-predicate (PredicateNode "ball_99_color")) =
;         (PredicateNode "ball_99_color")
; This example seems stupid, but this function can find grounded
; Frames for Frame instances with VariableNodes and/or WordInstanceNodes
; as elements values, for example.
(define (get-grounded-frame-instance-predicate predicateNode)
  (let* (
        (frameType (get-frame-instance-type predicateNode))        
        (elementsPredicates (get-frame-element-instances predicateNode))
        (numberOfElements (length elementsPredicates))
        (candidates '())
        (chosenPredicate '())
        )
    
    (map
     (lambda (elementPredicate)
       (let* ((elementValue (get-frame-element-instance-value elementPredicate))
              (elementType (get-frame-element-instance-type elementPredicate))
              (groundedValue (get-grounded-element-value elementValue))             
              )
         (if (not (null? groundedValue))
             (map
              (lambda (candidate)              
                (let ((value (assoc-ref candidates candidate))
                      (tv (cog-tv (InheritanceLink candidate (DefinedFrameNode frameType))))
                      )
                  (if (eq? value #f)
                      (set! value 0)
                      )
                  ; only frames with truth value greater than 0 will be considered
                  (if (> (assoc-ref (cog-tv->alist tv) 'mean) 0)
                      (set! candidates (alist-cons candidate (+ value 1) (alist-delete candidate candidates)))
                      )
                  )
                )
              (get-frame-instances-given-an-element frameType elementType groundedValue)
              )
             ) ; if               
         )
       )
     elementsPredicates
     )

    ; only a frame instance which matched the same number of elements of the original frame
    ; can be considered valid. so, i'll pick the first
    (map
     (lambda (candidate)
       (if (and (null? chosenPredicate) (= (cdr candidate) numberOfElements))
           (set! chosenPredicate (car candidate))
           )
       )
     candidates
     )

    chosenPredicate
    )
  )


; Given a specific node as value, this function, will try to find a node wich 
; grounds the given one. For instance, a grounded node for a SemeNode is an
; ObjectNode (or a child of it)
;
; i.e. (get-grounded-element-value (WordNode "blue")) = (ConceptNode "blue")
;
(define (get-grounded-element-value value)
    (if (or (null? value)
            (equal? (cog-type value) 'VariableNode)
        )

        value

        (let ( (groundedValue '())
             )
             ; If it is a WordInstanceNode, try to find a reference resolution 
             ; SemeNode
             (if (equal? (cog-type value) 'WordInstanceNode)
                 (let ( (refLinks
                            (cog-get-link 'ReferenceLink 'SemeNode value)
                        )
                      )
                      (if (not (null? refLinks))
                          (set! groundedValue
                              (gar (car refLinks))
                          )
                      ); if    
                 ); let             
             ); if
        
             ; If there's no grounded SemeNode, then try to find a corresponding 
             ; node (ConceptNode)
             (if (null? groundedValue)
                 (let ( (name (cog-name value))
                      )
                      (if (and (> (string-length name) 1)
                               (string=? "#" (substring name 0 1))
                          )

                          (set! groundedValue
                              (get-corresponding-node (substring name 1 (string-length name)))
                          )

                          (let ( (wordNode (get-word-node value)) 
                               )
                               (if (not (null? wordNode))
                                   (set! groundedValue
                                       (get-corresponding-node (cog-name wordNode))
                                   )
                               )
                          ); let
                      ); if
                 ); let
             ); if

             ; return the grounded value
             groundedValue
        ); let
    ); if  
)

; Given a Frame instance (PredicateNode) this function tries to find a grounded 
; frame that matches the given Frame instance
(define (match-frame frame_instance)
    (let ( (candidates '())
           (evaluatedElements 0)
           (frameType (get-frame-instance-type frame_instance))
           (elementsStrength '())
         )

         (map ; inspect all the frame elements
             (lambda (frame_element_instance)
                 (let* ( (value (get-frame-element-instance-value frame_element_instance))
                         (groundedValue (get-grounded-element-value value))
                         (elementType (get-frame-element-instance-type frame_element_instance))
                         (elementName (get-frame-element-name elementType))
                         (elementTypeNode (get-frame-element-node frameType elementName) )
                         (elementsCandidates '())
                       )
             
                       ; check if the element has a Variable as its value
                       (if (equal? 'VariableNode (cog-type value))
                           ; ok, it has a variable and must be matched against a stored frame
                           (map  ; retrieve from AT all elements of the same type
                               (lambda (inheritanceLink)
                                   (let* ( (candidate (gar inheritanceLink ))
                                           (frameInstancePredicate
                                               (gar (car (cog-filter-incoming 'FrameElementLink candidate)))
                                           )
                                         )
                                         ; only PredicateNodes are welcome
                                         (if (equal? 'PredicateNode (cog-type candidate))
                                             (map                         
                                                 (lambda (evalLink)
                                                     (let* ( (elementValue (car (gdr evalLink)))
                                                             (valueType (cog-type elementValue))
                                                           )
                                                           (if (or (equal? 'SemeNode valueType)
                                                                   (equal? 'ConceptNode valueType)
                                                               )
                                                               (set! elementsCandidates
                                                                   (append elementsCandidates (list frameInstancePredicate))
                                                               )
                                                               (set! elementsStrength 
                                                                   (append elementsStrength
                                                                       (list (cons frameInstancePredicate 
                                                                                   (get_truth_value_mean (cog-tv inheritanceLink) )
                                                                             )
                                                                       )
                                                                   )
                                                               ); set!
                                                           ); if
                                                     ); let*
                                                 ); lambda

                                                 (cog-filter-incoming 'EvaluationLink candidate)
                                             ); map
                                         ); if
                                   ); let*
                               ); lambda

                               (cog-filter-incoming 'InheritanceLink elementTypeNode)
                           ); map

                           (map ; else
                               (lambda (evalLink)
                                   (let* ( (candidate (gar evalLink))
                                           (candidateType (cog-type candidate))
                                         )
                                         (if (and (equal? 'PredicateNode candidateType) 
                                                  (not (null? (cog-link 'InheritanceLink 
                                                                         candidate
                                                                         elementTypeNode
                                                              )
                                                       )
                                                  ) 
                                             ); and
                                             (let ( (frameInstancePredicate 
                                                        (gar (car (cog-filter-incoming 'FrameElementLink candidate))))
                                                  )
                                                  (if (not (equal? frameInstancePredicate frame_instance))
                                                      (begin
                                                          (set! elementsCandidates 
                                                              (append elementsCandidates (list frameInstancePredicate))
                                                          )
                                                          (set! elementsStrength 
                                                              (append elementsStrength 
                                                                  (list (cons frameInstancePredicate 
                                                                              (get_truth_value_mean 
                                                                                  (cog-tv (cog-link 'InheritanceLink candidate elementTypeNode)) 
                                                                              ) 
                                                                        )
                                                                  )
                                                              )
                                                          ); set!
                                                      ); begin
                                                  ); if
                                             ); let
                                          ); if
                                   ); let*
                               ); lambda

                               (cog-filter-incoming 'EvaluationLink groundedValue)
                           ); map
                       ); if

                       (if (= evaluatedElements 0)
                           (set! candidates elementsCandidates)
                           (set! candidates 
                               (filter 
                                   (lambda (x) (member x elementsCandidates))
                                   candidates
                               )
                           ); set!
                       ); if
                 ); let*
       
                (set! evaluatedElements 1)
             ); lambda

             (get-frame-element-instances frame_instance)
         ); map

         ; sort by the candidate tv mean strength. strongest first.
         (sort candidates 
               (lambda (x y) 
                   (> (assoc-ref elementsStrength x) (assoc-ref elementsStrength y)
                   )
               )
         ); sort
    ); let
)

; NOTE: This function is not used; and is obsoleted by match-frame above, which uses PLN rather than Linas's query code.

; When a Frame instance with VariableNodes in its elements
; is used in a PatternMatching process to find a grounded Frame Instance,
; it is necessary to build a customized ImplicationLink to conclude that task.
; This function receives a PredicateNode which represents a Frame instance
; as argument and build an ImplicationLink to help us to find the values
; of the its elements that are marked as VariableNodes
(define (build-implication-link predicateNode)
  (let ((variableCounter 1)
  (variablesDeclaration '())
  (elementsDeclaration '())
        (frameType (get-frame-instance-type predicateNode))
  )
    (map
     (lambda (predicate)
       (let* ((value (get-frame-element-instance-value predicate))
        (groundedValue (get-grounded-element-value value))
              (variable? (and value (equal? 'VariableNode (cog-type value))))
              (elementNode (get-frame-element-node frameType (get-frame-element-name (get-frame-element-instance-type predicate) )))
       )
         (set! variablesDeclaration (append variablesDeclaration (list
      (TypedVariableLink
             (VariableNode (string-append "$var" (number->string variableCounter)))
             (VariableTypeNode "PredicateNode")
             ) 
            )))
         (if variable?
             (begin
               (set! groundedValue value)
               (set! variablesDeclaration (append variablesDeclaration (list
                  (TypedVariableLink
                   value
                   (ListLink
                    (VariableTypeNode "SemeNode")
                    (VariableTypeNode "ConceptNode")
                    )
                   )
                  )))
               ) ; begin
             ) ; if
         
         (set! elementsDeclaration (append elementsDeclaration (list         
      (FrameElementLink
             (VariableNode "$var0")
             (VariableNode (string-append "$var" (number->string variableCounter) ))
             )
            (InheritanceLink
             (VariableNode (string-append "$var" (number->string variableCounter) ))
             elementNode
             )
            (EvaluationLink
             (VariableNode (string-append "$var" (number->string variableCounter) ))
             groundedValue
             )
            ) ) )
   (set! variableCounter (+ variableCounter 1))
   ) ; let
       ) ; lambda
     (get-frame-element-instances predicateNode)
     )

    (BindLink
     (ListLink
      variablesDeclaration
      (TypedVariableLink
       (VariableNode "$var0")
       (VariableTypeNode "PredicateNode")
       )
      )
     (ImplicationLink
      (AndLink
       (InheritanceLink
  (VariableNode "$var0")
  (DefinedFrameNode (get-frame-instance-type predicateNode))
  )
       elementsDeclaration
       )
      (EvaluationLink
       (PredicateNode "groundedFrame")
       (ListLink
        (VariableNode "$var0")
        (let ((vars '()))
          (do ((i 1 (+ i 1)))
              ((= i variableCounter ) vars)
            (set! vars (append vars (list (VariableNode (string-append "$var" (number->string i) )))))
            )
          )
        )       
       )
      )
     )

    ) ; let
  )

; Prototype. Converts all frame instances from the messy format into the elegant format. Seems to be essential for
; PLN to be able to handle it with reasonable efficiency.
(define convert-frames
    (BindLink
     (ListLink
      (TypedVariableLink
       (VariableNode "$frameInstance")
       (VariableTypeNode "PredicateNode")
      )
      (VariableNode "$frameElementInstance")
;      (VariableNode "$frame")
      (VariableNode "$frameElement")
      (TypedVariableLink
       (VariableNode "$value")
       (ListLink
        (VariableTypeNode "ConceptNode")
        (VariableTypeNode "SemeNode")
        ; Do it on the raw WordInstanceNode version (ungrounded version) too. Needed by frames-to-implication.
        (VariableTypeNode "WordInstanceNode")
       )
      )
     )
     (ImplicationLink
      (AndLink
;            (InheritanceLink
;          (VariableNode "$frameInstance")
;          (VariableNode "$frame")
;            )
      (FrameElementLink
             (VariableNode "$frameInstance")
             (VariableNode "$frameElementInstance")
             )
            (InheritanceLink
             (VariableNode "$frameElementInstance")
             (VariableNode "$frameElement")
             )
            (EvaluationLink
             (VariableNode "$frameElementInstance")
             (VariableNode "$value")
            )
      )

      (EvaluationLink (VariableNode "$frameElement")
        (ListLink
            (VariableNode "$frameInstance")
            (VariableNode "$value")
        )
      )

     ) ; Implication
    ) ; VarScope
)

(define frames-to-inh
    (BindLink
     (ListLink
      (TypedVariableLink
       (VariableNode "$frameInstance")
       (VariableTypeNode "PredicateNode")
      )
      (TypedVariableLink
       (VariableNode "$entity")
       (ListLink
        (VariableTypeNode "ConceptNode")
        (VariableTypeNode "SemeNode")
       )
      )
      (TypedVariableLink
       (VariableNode "$attribute")
       (ListLink
        (VariableTypeNode "ConceptNode")
        (VariableTypeNode "SemeNode")
       )
      )
     )
     (ImplicationLink
      (AndLink
       (EvaluationLink (DefinedFrameElementNode "#Attributes:Entity")
         (ListLink
             (VariableNode "$frameInstance")             
             (VariableNode "$entity")
         )
       )
       (EvaluationLink (DefinedFrameElementNode "#Attributes:Attribute")
         (ListLink
             (VariableNode "$frameInstance")
             (VariableNode "$attribute")
         )
       )
      ) ; And
      (InheritanceLink (VariableNode "$entity") (VariableNode "$attribute"))
     ) ; Implication
    ) ; VarScope
)

(define (find-frame-elements-new-eval-links frameInstance)
    (let ( (generalFrameTemplate
               (EvaluationLink (VariableNode "$frameElement")
                 (ListLink                     
                     frameInstance
                     (VariableNode "$value")
                 )
               )
           )
         (variables
          (ListLink
               (TypedVariableLink
                (VariableNode "$value")
                (ListLink
                 (VariableTypeNode "ConceptNode")
                 (VariableTypeNode "SemeNode")
                )
               )
               (VariableNode "$frameElement")
          )
         )
         )
         (lookup-atoms variables generalFrameTemplate)
    )
)

; Lookup the query atom (can contain variables).
(define (lookup-atoms variables query)
    (let ((implication
         (BindLink
            variables
            (ImplicationLink
                query
                query
            )
         )
         ))
         
         ; cog-bind returns a ListLink of results, so get the list not the ListLink
         (cog-outgoing-set (cog-bind implication))
     )
)

; TODO: this function really shouldn't have to be this long.
(define (find-frame-instance-for-word-instance wordInstance)
    ; given:
    ; (WordInstanceNode eats@c19cfa35-0d75-4762-ae96-f960745d777f)
    ; find:
    ; (PredicateNode eats@c19cfa35-0d75-4762-ae96-f960745d777f_Ingestion)
    ; and without including:
    ; (PredicateNode eats@c19cfa35-0d75-4762-ae96-f960745d777f_Ingestion_Ingestor)
    
    ; We want it to find the grounded version of the frame instance, which starts with G_
    
    ; (InheritanceLink PredicateNode DefinedFrameNode)
    (define variables
        (ListLink
            (TypedVariableLink (VariableNode "frameInstance") (VariableTypeNode "PredicateNode"))
            (TypedVariableLink (VariableNode "frame") (VariableTypeNode "DefinedFrameNode"))
        )
    )
    (define result '())
    (let
        ((all-frame-instances
            (lookup-atoms variables
                (InheritanceLink (VariableNode "frameInstance") (VariableNode "frame")))
        ))

        (map
            (lambda (instanceLink)
                (let* ((instance (gar instanceLink))
                      (instanceFullName (cog-name instance))
                      )
                      (if (string=? "G_" (substring instanceFullName 0 2))
                          (let* ( (instanceRestName (substring instanceFullName 2) )
                              (underscoreIndex (list-index (lambda (char) (char=? char #\_)) (string->list instanceRestName) ))
                              (correspondingWordNodeName (substring instanceRestName 0 underscoreIndex))
                              )
                              (if (and
                                      (string=? correspondingWordNodeName (cog-name wordInstance))
                                      ; Reuse this ~hack for now, to make sure it's only one.
                                      (not (member (get-frame-instance-type instance) invalid-question-frames))
                                  )
                                  (set! result instance)
                              )
                          ) ; let*
                      )
                )
            )
            all-frame-instances
        )
    )
    
    result
)

; Called by the postcondition of frames-to-implication. Creates an ImplicationLink between (all elements of) two frames.
; The premise will later be an and/or/whatever link possibly containing multiple things. Same with the conclusion.
; Both of them should be a WordInstanceNode so that this method can find the right frames to connect.
(define (convert-frame-to-implication premise conclusion)
    (ImplicationLink (stv 1 1)
        (AndLink
            (find-frame-elements-new-eval-links (find-frame-instance-for-word-instance premise)))
        (AndLink
            (find-frame-elements-new-eval-links (find-frame-instance-for-word-instance conclusion)))
    )
)

; This format will only support one premise and conclusion. Though the Frames output simply has separate Frame instances
; when there are multiple conclusions, so this rule will simply be triggered multiple times.
(define frames-to-implication
    (BindLink
     (ListLink
      (TypedVariableLink
       (VariableNode "$frameInstance")
       (VariableTypeNode "PredicateNode")
      )
      (TypedVariableLink
       (VariableNode "$premise")
       (VariableTypeNode "WordInstanceNode")
      )
      (TypedVariableLink
       (VariableNode "$conclusion")
       (VariableTypeNode "WordInstanceNode")
      )
     )
     (ImplicationLink
      (AndLink
       (EvaluationLink (DefinedFrameElementNode "#Contingency:Determinant")
         (ListLink
             (VariableNode "$frameInstance")             
             (VariableNode "$premise")
         )
       )
       (EvaluationLink (DefinedFrameElementNode "#Contingency:Outcome")
          (ListLink
              (VariableNode "$frameInstance")             
              (VariableNode "$conclusion")
          )
       )
      ) ; AND
      (ExecutionLink
          (GroundedSchemaNode "scm:convert-frame-to-implication")
          (ListLink
              (VariableNode "$premise")
              (VariableNode "$conclusion")
          )
      )    
     ) ; Implication
    ) ; VariableScope
)

; Just a prototype for now. Uses a possibly-obsolete approach. For every Frame element, looks up an InheritanceLink, FrameElementLink and EvaluationLink.
(define (build-query_alt predicateNode)
  (let ((variableCounter 1)
  (variablesDeclaration '())
  (elementsDeclaration '())
        (frameType (get-frame-instance-type predicateNode))
  )
    (map
     (lambda (predicate)
       (let* ((value (get-frame-element-instance-value predicate))
        (groundedValue (get-grounded-element-value value))
              (variable? (and value (equal? 'VariableNode (cog-type value))))
              (elementNode (get-frame-element-node frameType (get-frame-element-name (get-frame-element-instance-type predicate) )))
       )
         (set! variablesDeclaration (append variablesDeclaration (list
      (TypedVariableLink
             (FWVariableNode (string-append "$var" (number->string variableCounter)))
             (VariableTypeNode "PredicateNode")
             ) 
            )))
         (if variable?
             (begin
               (set! groundedValue value)
               (set! variablesDeclaration (append variablesDeclaration (list
                  (TypedVariableLink
                   value
                   (ListLink
                    (VariableTypeNode "SemeNode")
                    (VariableTypeNode "ConceptNode")
                    )
                   )
                  )))
               ) ; begin
             ) ; if
         
         (set! elementsDeclaration (append elementsDeclaration (list
      (AndLink ; Nested AndLink because of PLN's current limitations
       (FrameElementLink
        (FWVariableNode "$var0")
        (FWVariableNode (string-append "$var" (number->string variableCounter) ))
        )
       (InheritanceLink
        (FWVariableNode (string-append "$var" (number->string variableCounter) ))
        elementNode
        )
       (EvaluationLink
        (FWVariableNode (string-append "$var" (number->string variableCounter) ))
        groundedValue
       )
      )
            ) ) )
   (set! variableCounter (+ variableCounter 1))
   ) ; let
       ) ; lambda
     (get-frame-element-instances predicateNode)
     )

;    (BindLink
;     (ListLink
;      variablesDeclaration
;      (TypedVariableLink
;       (Node "$var0")
;       (VariableTypeNode "PredicateNode")
;       )
;     )
;     (ImplicationLink
      (AndLink
       elementsDeclaration
       (InheritanceLink
  (FWVariableNode "$var0")
  (DefinedFrameNode (get-frame-instance-type predicateNode))
  )
      )
;     )

    ) ; let
)

; Just a prototype for now. Takes a Frame instance, and Produces a query for PLN, to look up all matching Frame instances (including all of their Frame elements).
(define (build-query predicateNode)
  (let ((variableCounter 1)
  (variablesDeclaration '())
  (elementsDeclaration '())
        (frameType (get-frame-instance-type predicateNode))
  )
    (map
     (lambda (predicate)
       (let* ((value (get-frame-element-instance-value predicate))
        (groundedValue (get-grounded-element-value value))
              (variable? (and value (equal? 'VariableNode (cog-type value))))
              (elementNode (get-frame-element-node frameType (get-frame-element-name (get-frame-element-instance-type predicate) )))
       )
         (set! variablesDeclaration (append variablesDeclaration (list
      (TypedVariableLink
             (FWVariableNode (string-append "$var" (number->string variableCounter)))
             (VariableTypeNode "PredicateNode")
             ) 
            )))
         (if variable?
             (begin
               (set! groundedValue (FWVariableNode (string-append "$var" (number->string variableCounter))))
               ) ; begin
             ) ; if
         
         (set! elementsDeclaration (append elementsDeclaration (list
            (EvaluationLink (DefinedFrameElementNode (get-frame-element-instance-type predicate)) ; FrameElement
                (ListLink
                    (FWVariableNode "$var0") ; FrameInstance
                    groundedValue ; Value
                )
            )
         ) ) )
   (set! variableCounter (+ variableCounter 1))
   ) ; let
       ) ; lambda
     (get-frame-element-instances predicateNode)
     )

;    (BindLink
;     (ListLink
;      variablesDeclaration
;      (TypedVariableLink
;       (Node "$var0")
;       (VariableTypeNode "PredicateNode")
;       )
;     )
;     (ImplicationLink

;      (AndLink
;       elementsDeclaration
;       (InheritanceLink
;  (FWVariableNode "$var0")
;  (DefinedFrameNode (get-frame-instance-type predicateNode))
;  )
;      )
      (AndLink
          elementsDeclaration
      )
;     )

    ) ; let
)


; This function receives as argument a list of Frame instances (PredicateNodes) 
; and returns a list of PredicateNodes that is grounded versions of the given
; ones. This is part of the "old" approach, which looked up Atoms directly rather 
; than using PLN. Still leaving it available for now.
;
(define (find-grounded-frame-instances-predicates frame_instances)
    (let ( (finalFrames '()) 
         )
         (map ; ok it is a question, so handle it
              ; start by creating new frames with SemeNodes as element values instead of nouns/pronouns WINs
              ; questionFrames
             (lambda (frame_instance)
                 (frame-preprocessor frame_instance)

                 (if (frame-instance-contains-variable? frame_instance)
                     (let ( (groundedFrameInstances (match-frame frame_instance)) 
                          )
                          (if (not (null? groundedFrameInstances))
                              (set! finalFrames
                                  (append finalFrames (list (car groundedFrameInstances )))
                              )
                          ); if
                     ); let

                     (let ( (groundedFrameInstance 
                                (get-grounded-frame-instance-predicate frame_instance))
                          )
                          (if (not (null? groundedFrameInstance))                                    
                              (set! finalFrames 
                                  (append finalFrames (list groundedFrameInstance ))
                              )
                          )
                     ); let
                 ); if
             ); lambda

;           (if (list? (car frame_instances)) (car frame_instances) frame_instances)
             (if (and (pair? frame_instances)
                      (list? (car frame_instances))
                 )
                 (car frame_instances)
                 frame_instances
             )
;    frame_instances
         ); map

         ; return value
         finalFrames
    ); let
)

; This function receives as argument a list of PredicateNodes that
; represents instances of Frames and returns a list of Frame instances that match.
; Uses PLN. This is an alternative to the find-grounded-frame-instances-predicates and match-frame methods.
(define (find-grounded-frame framesPredicates)
  (let ((finalFrames '()))
    (map ; ok it is a question, so handle it
         ; start by creating new frames with SemeNodes as element values instead of nouns/pronouns WINs
         ;questionFrames
      (lambda (predicate)
        (frame-preprocessor predicate)
        (begin
                (cog-bind convert-frames)
                (cog-bind frames-to-inh)
                ; Convert any Contingency frames into ImplicationLinks.
                (cog-bind frames-to-implication)

                (let* ( ( tmp (pln-bc (build-query predicate) 2000) )  ; An AndLink
                     )
                  (if (not (equal? (cog-handle tmp) (cog-undefined-handle)))
                    (let* (
                          ( groundedFrameInstance (gar (car (gdr (gar tmp)))) ) ; This extracts the frame instance (*gasp*)
                         )
                         (set! finalFrames (append finalFrames (list groundedFrameInstance )))
                    )
                  )
                )
        )
      ) ; lambda
    ;(if (list? (car framesPredicates)) (car framesPredicates) framesPredicates) 
     framesPredicates
    ) ; map
    finalFrames
  ) ; let
)

; Used by store-fact to make a new frame instance (with ConceptNodes instead of WordInstanceNodes)
(define (instantiate-frame type instanceName elements)
  (let ((frameElements '())
        (frameElementsNodes '())
        (isFrameInstance? #f)
        )
    (map
     (lambda (elementType)
       (let ((elementName (get-frame-element-name (cog-name elementType))))
         (set! frameElements (append frameElements (list elementName)))
         (set! frameElementsNodes (append frameElementsNodes (list (cons elementName elementType))))
         )
       )
     (get-frame-elements type)
     )
    
    (map
     (lambda (element)
       (let ((name (car element))
             (value (cdr element)))

         (if (member name frameElements)                            
             (let ((elementPredicateNode (PredicateNode (string-append instanceName "_" name ))))
               (InheritanceLink (stv 1 1) (cog-new-av 0 1 0)
                elementPredicateNode
                (assoc-ref frameElementsNodes name)
                )
               (FrameElementLink (stv 1 1) (cog-new-av 0 1 0)
                (PredicateNode instanceName)
                elementPredicateNode
                )
               (EvaluationLink (stv 1 1) (cog-new-av 0 1 0)
                elementPredicateNode
                value
                )               
               (set! isFrameInstance? #t)
               ) ; let
             ) ; if

         ) ; let
       ) ; lambda
     elements
     )

    (if isFrameInstance?
        (InheritanceLink (stv 1 1) (cog-new-av 0 1 0)
         (PredicateNode instanceName)
         (DefinedFrameNode type)         
         )
        '()
        )    
    )
  )

;;; Core functions

; This function execute the whole process of reference resolution. 
;
; @return The output of this function is a list of WordInstanceNodes 
;         (nouns and pronouns) and one SemeNode for each WordInstanceNode that
;         was chosen by the reference resolution process. 
;
;         A list of ReferenceLink with the format below, 
;
;         ReferenceLink
;             SemeNode
;             WordInstanceNode
;
; @note It must be called after a new sentence has been loaded into the AtomTable.
;       Reference resolution is a process that uses perceptions and predicates, 
;       built using the state of the environment which contains the agent, to
;       identify the real elements that were mentioned by someone in a given 
;       sentence.
;       
;       You can use 'get-grounded-element-value' function to retrieve the 
;       grounded SemeNode given a WordInstanceNode.
;
; @see  http://wiki.opencog.org/w/ReferenceResolution_%28Embodiment%29
;
(define (resolve-reference)
    (let ( (objects '()) ; a list of (WordInstanceNode . SemeNodes)
           (resolvedReferences '())
           (anaphoricSemeNodeStrength '() ) ; used by 'filter-by-strength'
           (groundedRulesCounter '() )      ; used by 'filter-by-strength''
         )    

         (set! word-node-seme-nodes-cache '())

         ; Step 1. retrieve all objects, from the latest sentence, to be evaluated
         ;         objects is a list of (WordInstanceNode . SemeNodes)
         (map 
             (lambda (win)
                 (let ( (semeNodes '())
                        (semeNodesCandidates (get-candidates-seme-nodes win))
                      )

                      (map
                          (lambda (candidateSemeNodesList)          
                              (let ( (strength (car candidateSemeNodesList) )
                                     (seme_node_list (cdr candidateSemeNodesList) )
                                   )

                                   (map
                                       (lambda (semeNode)
                                           (set! semeNodes
                                               (append semeNodes (list semeNode) ) 
                                           )

                                           ; keep the greater strength suggestion
                                           (let ( (oldSuggestion
                                                      (assoc semeNode anaphoricSemeNodeStrength)
                                                  )
                                                )
                                                (cond 
                                                    ( (and oldSuggestion
                                                           (> strength (cdr oldSuggestion) ) 
                                                      )
                                                      (set! anaphoricSemeNodeStrength 
                                                            (alist-delete semeNode anaphoricSemeNodeStrength) 
                                                      )
                                                      (set! oldSuggestion #f)
                                                    )
                                                )
                    
                                                (if (not oldSuggestion)
                                                    (set! anaphoricSemeNodeStrength 
                                                        (append anaphoricSemeNodeStrength 
                                                            (list (cons semeNode strength) ) 
                                                        ) 
                                                    )
                                                )
                                           
                                           ); let                 

                                           (set! groundedRulesCounter 
                                                (append groundedRulesCounter (list (cons semeNode 0) ) )
                                           )
                                       ); lambda

                                       seme_node_list
                                   ); map

                              ); let
                          ); lambda
                          semeNodesCandidates
                      ); map
 
                      (if (not (null? semeNodes))
                          (set! objects 
                                (append objects 
                                         (list (cons win (delete-duplicates semeNodes)))
                                )
                          )
                      )
 
                 ); let
             ); lambda

             (get-latest-word-instance-nodes)
         ); map

         ; Step 2. use all sorts of rules to filter the objects list, i.e. reserve
         ;         only suitable SemeNodes (defined by rules) for each WordInstanceNode
         (map
             (lambda (rule)
                 (map
                     (lambda (winAndSemesListLink)
                         (cond
                             ( (not (null? winAndSemesListLink) )
                               (let* ( (winAndSemes (cog-outgoing-set winAndSemesListLink) )
                                       (win (car winAndSemes) )
                                       (semes (cdr winAndSemes) )
                                     )
       
                                     ; For each SemeNode, udpate its RulesCounter
                                     ; i.e. (SemeNode . Number of rules)
                                     ; (delete old one and create new one)
                                     (map
                                         (lambda (semeNode)
                                             (let ( (rulesCounter
                                                        (+ (assoc-ref groundedRulesCounter semeNode) 1)
                                                    ) 
                                                  )
                                                  (set! groundedRulesCounter 
                                                        (alist-delete semeNode groundedRulesCounter)
                                                  )
                                                  (set! groundedRulesCounter
                                                        (alist-cons semeNode rulesCounter groundedRulesCounter)
                                                  )
                                             )
                                         );lambda
                                         semes
                                     ); map

                                     ; remove those SemeNodes from objects that
                                     ; is not present in winAndSemes, whcih must
                                     ; not be present in the answer
                                     (set! objects (filter-objects objects winAndSemes) )
                               ); let*
                             )
                         ); cond
                     ); lambda

                     (cog-outgoing-set (cog-bind rule) )
                ); map
        
             ); lambda

             reference-resolution-rules
         ); map

         ; Step 3. filter objects even further
         
         ; filter by the number of satisfied rules
         (set! objects (filter-by-strength objects groundedRulesCounter) )

         ; filter the objects by their strengths given by the anaphora resolution
         ; if there is no anaphoric suggestion, the objects list will remains the same
         (set! objects (filter-by-strength objects anaphoricSemeNodeStrength) )
    
         ; filter by diatance, i.e., pick up the nearest SemeNode for each 
         ; WordInstanceNode
         (map
             (lambda (instance)
                 (let ( (win (car instance) )
                        (semeNode 
                            (get-nearest-candidate (cdr instance) ) 
                        )
                      )

                      (set! resolvedReferences 
                          (append resolvedReferences 
                              (list (ReferenceLink (stv 1 1) semeNode win))
                          )
                      )
                 )       
             ); lambda

             objects
         ); map
    
         resolvedReferences
    ); let
)

; This function is responsible for identifying the presence of Frames instance
; in the most recent parsed sentence and tries to recognize given commands. 
;
; @return a list of commands (ExecutionLinks). It also creates an
;         latestAvatarRequestedCommands predicate connecting to these commands. 
; 
; @note When a sentence containing an imperative verb is parsed. Frames that 
;       represents the given command can be identified via command-resolution-rules,
;       and then transformed into a grounded command by createActionCommand funciton. 
;
; @see  http://wiki.opencog.org/w/CommandResolution_%28Embodiment%29
;
(define (resolve-command)
    (let ( (commands '() ) 
         )
         ; set the truth value of the latests eval links to false
         ; TODO: why not just remove these EvaluationLinks???
         (map
             (lambda (evalLink)
                 (cog-set-tv! evalLink (stv 0 0))
             )

             (cog-get-link 
                 'EvaluationLink 'ListLink (PredicateNode "latestAvatarRequestedCommands")
             )
         ); map

         (map
             (lambda (rule)
                 (map
                     (lambda (candidateCommand)
                         (if (not (member candidateCommand commands))
                             (set! commands 
                                 (append commands (list candidateCommand))
                             )
                         )
                     )
                     (cog-outgoing-set (cog-bind rule) )
                 )
             )

             command-resolution-rules
         ); map

         (cond ( (not (null? commands) )
                 (EvaluationLink (stv 1 1)
                     (PredicateNode "latestAvatarRequestedCommands")
                     (ListLink
;                         (unique-list commands)
                         (delete-duplicates commands)
                     )
                 )
               )
         ); cond

         commands
    ); let
)

; This function is responsible for getting a bunch of Frames instances
; that represents a question made by another agent and start a pattern
; matching process to find the answer inside the agent's AtomSpace.
; The matching starts by getting all parses of the sentence that 
; represent the questions. Each parse has a list of frames.
; The parse which matches a greater number of frames will be chosen
; as the question answer.
(define (answer-question)
    (let ( (chosenAnswer '() )
           (question? #f)
           (questionType '())
        
           (use-pln #t)
         )
        
         (map
             (lambda (parse)
                 ; Use unique-list because sometimes the Frames include bogus extra copies of the same frame element+value
                 (let ( (frame_instances 
;                          (unique-list (get-latest-frame-instances (get-latest-word-instance-nodes parse))) 
                            (delete-duplicates (get-latest-frame-instances
                                                   (get-latest-word-instance-nodes parse)
                                               )
                            )
                        )
                        (questionParse? #f)
                        (questionFrames '())
                      )

                      ; Note: question frames aren't grounded at this point. get-grounded-element-value is used by the matching functions
                      ; before doing the lookup (or PLN inference). store-fact also does this for statement sentences.
                      ; i.e. at this point it's all WordInstanceNodes rather than ConceptNodes and SemeNodes.
                      (map
                          (lambda (frame_instance)
                              (if (string=? (get-frame-instance-type frame_instance) "#Questioning")
                                  (begin
                                      (let ( (manner 
                                                 (get-frame-element-instance-value
                                                     (get-frame-element-instance frame_instance "Manner")
                                                 )
                                             )
                                           )
                                           (cond ( (not (null? manner))

                                                   (set! questionType (cog-name manner))
                                                   (set! question? #t)
                                                   (set! questionParse? #t)
                                                 )
                                           )
                                      ); let
                                  ); begin

                                  (if (and (not (member (get-frame-instance-type frame_instance)
                                                        invalid-question-frames
                                                )
                                           )

                                           ; Get rid of the possibly bogus Attributes:Attribute(be@..., truth-query) FEs, 
                                           ; which seem incorrect -- JaredW
                                           (or (null? (get-frame-element-instance-value 
                                                          (get-frame-element-instance frame_instance "Attribute")
                                                      )
                                               ); null?

                                               (not (equal? 'ConceptNode 
                                                             (cog-type (get-frame-element-instance-value 
                                                                           (get-frame-element-instance frame_instance "Attribute")
                                                                       )
                                                             ) 
                                                    )
                                               )

                                               (not (equal? "#truth-query" 
                                                            (get-frame-element-instance-value
                                                                (get-frame-element-instance frame_instance "Attribute")
                                                            )
                                                    )
                                               ); not
                                           ); or
                                      ); and

                                      (set! questionFrames
                                          (append questionFrames (list frame_instance))
                                      )
                                  ); if
                              ); if
                          ); lambda

                          frame_instances
                      ); map

                      ; todo: various ways of cleaning up the following code.
                      (if use-pln
                          (if questionParse?
                              (let* ( (groundedPredicates (find-grounded-frame questionFrames) )
                                    )
                                    (if (> (length groundedPredicates) 0)
                                        (set! chosenAnswer groundedPredicates)
                                    )
                              )
                          )

                          ; else use the old version
                          (begin 
                              (if questionParse?
                                  (let* ( (numberOfQuestionFrames (length questionFrames))
                                          (groundedPredicates 
                                              (find-grounded-frame-instances-predicates questionFrames)
                                          )
                                          (numberOfGroundedPredicates (length groundedPredicates))
                                          (balance (- numberOfQuestionFrames numberOfGroundedPredicates))
                                        )
;                                       (if (and (> numberOfQuestionFrames 0) 
;                                                (= balance 0) 
;                                                (or (null? chosenAnswer)
;                                                    (> numberOfGroundedPredicates (car chosenAnswer))
;                                                )
;                                           )
                                        ; Change by JaredW: only accept if _all_ frames are matched.
                                        (if (and (> numberOfQuestionFrames 0)
                                                 (= balance 0)
                                            )
                                            (set! chosenAnswer
                                                (cons numberOfGroundedPredicates groundedPredicates)
                                            )
                                        )
                                   ); let*
                              ); if
           
                              ; Make it just the actual frame instances, consistent with the PLN version above.
                              (if (not (null? chosenAnswer) )
                                  (set! chosenAnswer (cdr chosenAnswer))
                              )
                          ); begin
                      ); if
                 ); let
             ); lambda

             ; Get a list of ParseNodes of latest parsed sentence
             ;
             ; (ParseLink
             ;     (ParseNode "sentence@2d19c7e7-2e02-4d5e-9cbe-6772174f3f4d_parse_0")
             ;     (SentenceNode "sentence@2d19c7e7-2e02-4d5e-9cbe-6772174f3f4d")
             ; )
             (get-latest-parses)
         ); map

         (if question?
             (begin
                 ; first remove old predicates
                 (map
                     (lambda (evalLink)
                         (cog-set-tv! evalLink (stv 0 0))
                     )
                     (cog-get-link 'EvaluationLink 'ListLink (PredicateNode "latestQuestionFrames"))
                 ); map

                 ; then create a new one
                 (EvaluationLink (stv 1 1)
                     (PredicateNode "latestQuestionFrames")
                     (ListLink
                         (if (not (null? chosenAnswer) ) 
                             chosenAnswer
                             '() 
                         )
                     )
                 ); EvaluationLink

                 questionType ; Return the question type...
             ); begin

             '(); ... or else nothing.
         ); if
  
    ); let
)

; A fact is a set of Frames which describes something.
; It can be used as knownledge by the agent to answer
; questions or in any reasoning process.
; Basically, this function find a grounded frame instance
; for each Frame that composes the parsed sentence
; and set their av and tv values. It also replaces WordInstanceNodes
; with their normalised form.
(define (store-fact)
  (map
   (lambda (parse)
     (let ((negation? #f)
           (incomingPredicates 
            (get-latest-frame-instances 
             (get-latest-word-instance-nodes parse))))
       
       (map
        (lambda (predicate)
          (if (equal? (get-frame-instance-type predicate) "#Negation" )
              (set! negation? #t)
              )
          )
        incomingPredicates
        )
       (if negation?
           (map
            (lambda (groundedPredicate)
              (remove-frame-instance groundedPredicate)
              )
            (find-grounded-frame-instances-predicates incomingPredicates)
            )

           (map
            (lambda (predicate)
              (if (null? (match-frame predicate))
                  (let ((elements '()))
                    ; there is no grounded frame, so create a new one
                    (map
                     (lambda (elementPredicate)
                       (set! elements (append elements
                         (list (cons
                                (get-frame-element-name (get-frame-element-instance-type elementPredicate))
                                (get-grounded-element-value
                                 (get-frame-element-instance-value elementPredicate)
                                 )))))
                     )
                     (get-frame-element-instances predicate)
                   )
                   (instantiate-frame (get-frame-instance-type predicate) 
                                      (string-append "G_" (cog-name predicate)) elements ) 
                 ) ; let
               )
             )
             incomingPredicates
           ) ; map

       ) ; if
     ) ; let
   ) ; lambda
   (get-latest-parses)
  ) ; map

)

; Helper functions that receives a list of SenseNodeLinks
; and return a list containing the SenseNodes positioned
; at the index 0 in each link
(define (create-sense-node-list links)
  (if (null? links)
      '()
      (cons (gadr (car links)) (create-sense-node-list (cdr links)))
      )
  )

; This function retrieves the current elapsed time in seconds
; It works in two ways. If nothing is passed as argument
; it will return the current time. But if another tick was
; given, it will return the difference between the current
; and the given one
(define (get-tick . t)
  (let* ((now (gettimeofday))
         (seconds (+ (car now) (/ (cdr now) 1000000.0))))
    (if (null? t)
        seconds
        (- seconds (car t))
        )
    )
  )

; Given a list of WordNodes, it returns a list with two elements
; 1) an association list with a pair (SenseNode WordNode)
; 2) a SenseNode list
(define (get-word-sense-map wordNodes)
   (fold
    (lambda (wordNode result)
      (fetch-incoming-set wordNode)
      (let ((innerMap
             (fold
              (lambda (link elements)
                (list
                 (append (car elements) (list (cons (gadr link) wordNode)))
                 (append (cadr elements) (list (gadr link)))
                 )
                )
              (list '() '())
              (cog-get-link 'WordSenseLink 'WordSenseNode wordNode)
              )
             ))
        (list
         (append (car result) (car innerMap))
         (append (cadr result) (cadr innerMap))
         )
        )
      )
    (list '() '())
    wordNodes
    )
   )  

; This function makes use of the WordNet ontology
; to determine if a at least one word node of the given
; word nodes list is a child of the given parent word node.
; This function traverses the WordNet ontology, by following
; the InheritanceLinks and executes inferences of isA type
; to check the parentage
; It also has a timeout counter that will stop the current
; evaluation to avoid performance issues.
; The performance issues are mostly due to loading stuff from the database
; and, in some cases, the high number of word senses for a word.
(define (check-parentage parentWordNode wordNodes)
  (define expandedNodes '())
  (define inspectedNodes '())
  ; It seems to need more than 5 seconds though...
  (define timeout 5) ; measured in seconds.
  (define startTime (get-tick))
  (define candidateSenseNodes '())

  ; It's possible to make InheritanceLinks for every INdirect hypernym as well
  ; as the direct ones. e.g. using Linas's forward chainer. If you do so,
  ; set this to false so you don't need to do the recursion within this
  ; function.
  (define recursive #t)

  ; Fetching the Atoms from the database is a/the major overhead for this function.
  ; This wrapper only fetches the incoming set if it hasn't been already. That's only
  ; relevant in repeat calls to check-parentage.
  (define (maybe-fetch atom)
    (if (= 0 (length (cog-incoming-set atom)))
      (fetch-incoming-set atom)
      (cog-handle atom)
    )
  )

  (define (create-parent-sense-node-list links referenceNode)
    (if (null? links)
        '()
        (let ((parent (gadr (car links))))
          (if (not (equal? parent referenceNode))
              (cons parent (create-parent-sense-node-list (cdr links) referenceNode))
              (create-parent-sense-node-list (cdr links) referenceNode)
              )
          )
        )
    )

  ; Checks whether any of senseNodes is a member of candidateSenseNodes.
  ; candidateSenseNodes are the senses of the parentWordNode.
  (define (inspect-sense-nodes senseNodes)
    (if (or (null? senseNodes) (> (get-tick startTime) timeout))
        #f
        (if (member (car senseNodes) candidateSenseNodes)
            #t
            (inspect-sense-nodes (cdr senseNodes))
            )
        )
    )

  (define (inspect-sense-nodes-hierarchy senseNodes)
    (if (or (null? senseNodes) (> (get-tick startTime) timeout))
        #f
        (let ((senseNode (car senseNodes)))
          (if (member senseNode inspectedNodes)
              (inspect-sense-nodes-hierarchy (cdr senseNodes))
              (begin
                (set! inspectedNodes (append inspectedNodes (list senseNode)))
                (cond ((not (member senseNode expandedNodes))
                       (maybe-fetch senseNode)
                       (set! expandedNodes (append expandedNodes (list senseNode)))
                       )) ; cond
 
                (let* ((links (cog-get-link 'InheritanceLink 'WordSenseNode senseNode))
                       (parents (create-parent-sense-node-list links senseNode)))
                  (if (inspect-sense-nodes parents)
                      senseNode
                      (if recursive
                        (inspect-sense-nodes-hierarchy (append (cdr senseNodes) parents))
                        (inspect-sense-nodes-hierarchy (append (cdr senseNodes)))
                      )
                  )
                )                
              )
          )
        )
    )
  )
    
  (cons
   (if (member parentWordNode wordNodes)
       parentWordNode
       (begin
         (maybe-fetch parentWordNode)
         (set! expandedNodes (append expandedNodes (list parentWordNode)))
         (set! candidateSenseNodes 
               (create-sense-node-list
                (cog-get-link 'WordSenseLink 'WordNode parentWordNode)
                )
               )

         (if (null? candidateSenseNodes)
             #f
             ; get all sense nodes related to the given wordnodes
             (let* ((wordSenseMap (get-word-sense-map wordNodes))
                    (chosenSenseNode (inspect-sense-nodes-hierarchy (cadr wordSenseMap)))
                    )
               (if chosenSenseNode
                   (assoc-ref (car wordSenseMap) chosenSenseNode)
                   #f
                   )
               ) ; let*
             ) ; if
         ) ; begin
       )
   (get-tick startTime)
   )
)


; This function will execute a part-whole inference to check
; if one object is part of another one.
; Given a list of word nodes, that represents whole objects,
; this functions tries to figure out if the given part word node
; belongs to at least one whole word node.
; It uses the relation meronym/holonym of the WordNet to do
; the work. If the whole nodes doesn't match the current part,
; their parents will be evaluated and so one until the current
; inheritance level reaches the maxLevel (control variable).
; It also has a timeout counter that will stop the current
; evaluation to avoid performance issues.
(define (check-part-whole partWordNode wholeWordNodes)
  (define expandedNodes '())
  (define timeout 8) ; measured in seconds
  (define maxLevel 1) ; maximum inheritance levels
  (define startTime (get-tick))

  (define candidateSenseNodes '())
  (define wordSenseMap '())

  (define (get-holonyms senseNode)
    (cond ((not (member senseNode expandedNodes))
           (fetch-incoming-set senseNode)
           (set! expandedNodes (append expandedNodes (list senseNode)))
           ))
    (fold
     (lambda (link result)
       (let ((holonym (gadr link)))
         (if (not (equal? holonym senseNode))
             (cons holonym result)
             result
             )
         )
       )
     '()
     (cog-filter-incoming 'HolonymLink senseNode)
     )
    )

  (define (get-parents senseNode)
    (cond ((not (member senseNode expandedNodes))
           (fetch-incoming-set senseNode)
           (set! expandedNodes (append expandedNodes (list senseNode)))
           ))
    (fold
     (lambda (link result)
       (let ((parent (gadr link)))
         (if (not (equal? parent senseNode))
             (begin
               (if (not (assoc-ref wordSenseMap parent))
                   (set! wordSenseMap 
                         (append wordSenseMap 
                                 (list 
                                  (cons parent
                                        (assoc-ref wordSenseMap senseNode)))))
                   )
               (cons parent result)
               )
             result             
             )
         )
       )
     '()
     (cog-filter-incoming 'InheritanceLink senseNode)
     )
    )


  (define (inspect-sense-nodes-by-level senseNodes level)
    (let ((answer (inspect-sense-nodes senseNodes)))
      (if answer
          answer        
          (if (< level maxLevel)
              (begin
                (set! candidateSenseNodes 
                      ;;; Warning: this code block can take several
                      ;;; seconds to be processed. Perhaps a good
                      ;;; way of optimizing it is to create a 
                      ;;; variant of fetch-incoming-set function
                      ;;; that receives a list of atoms and buid
                      ;;; just one sql to retrieve all incoming
                      ;;; set in one operation
                      (delete-duplicates
                       (fold
                        (lambda (senseNode result)
                          (append result (get-parents senseNode))
                          )
                        '()
                        candidateSenseNodes
                        )
                       )
                      )
                (inspect-sense-nodes-by-level senseNodes (+ level 1))
                )
              #f
              )
          )
      )
    )
  
  (define (inspect-sense-nodes senseNodes)
    (if (null? senseNodes)
        #f
        (let ((senseNode (car senseNodes)))
          (cond ((not (member senseNode expandedNodes))
                (fetch-incoming-set senseNode)
                (set! expandedNodes (append expandedNodes (list senseNode)))
                ))
          (if (> (get-tick startTime) timeout)
              #f
              (let ((answer (find (lambda (x) (member x candidateSenseNodes))
                                  (get-holonyms senseNode))))
                (if answer
                    answer
                    (inspect-sense-nodes (cdr senseNodes))
                    )
                )                
              ) ; if
          )      
        )
    )
    
  (cons
   (if (member partWordNode wholeWordNodes)
       #f
       (begin
         (fetch-incoming-set partWordNode)        
         
         (if (> (get-tick startTime) timeout)
             #f
             (begin
               (set! wordSenseMap (get-word-sense-map wholeWordNodes))
               (set! candidateSenseNodes (delete-duplicates (cadr wordSenseMap)))
               (set! wordSenseMap (car wordSenseMap))
 

               (let ((senseNode
                      (inspect-sense-nodes-by-level
                       (create-sense-node-list 
                        (cog-filter-incoming 'WordSenseLink partWordNode)) 0)
                      ))
                 (if senseNode
                     (assoc-ref wordSenseMap senseNode)
                     #f
                     )
                 ) ; let
               )
             )
         )
       )
   (get-tick startTime)
   )
  
  )

; This return all the SemeNode associated with a given WordNode.
;
; If none SemeNodes was found, it will use the WordNet ontology to infer the 
; meaning of the given WordNode. Perhaps the given WordNode refers to some 
; SemeNode which really exists inside the agent's mind, but it is an abstraction.
; So, using the WordNet ontology is possible to determine if the mentioned 
; WordNode refers or not the existing SemeNode.
;
(define (get-seme-nodes-using-ontology wordNode)
    (define useWordNet #f) ; if true will use the WordNet ontology

    (if (assoc-ref word-node-seme-nodes-cache wordNode)
    
        ; If we can get corresponding SemeNodes from cache, return these SemeNodes
        (assoc-ref word-node-seme-nodes-cache wordNode)

        ; If we fail to get corresponding SemeNodes from cache, search in the AtomSpace
        (let ( (semeNodes (get-seme-nodes wordNode)) 
             )
             (if (or (not (null? semeNodes)) 
                     (not useWordNet)
                 )

                 ; If we found corresponding SemeNodes in AtomSpace, return them
                 semeNodes        

                 ; If we failed to get any SemeNode in AtomSpace, search in WordNet
                 (let ( (nodes ; contains all the grounded SemeNodes without duplications?
                        (delete-duplicates
                            (fold 
                                (lambda (semeNode result)
                                    (let ( (reference_link_list
                                               (cog-get-link 'ReferenceLink 'WordNode semeNode) 
                                           )
                                         )
                                         (if (null? reference_link_list)
                                             result
                                             (cons (gadr (car reference_link_list)) result)
                                         )
                                    )
                                )
                                '()
                                (cog-get-atoms 'SemeNode)
                            )
                        )
                        )
                      )            

                      (let ( (parentCheck (check-parentage wordNode nodes))
                           )

                           (if (car parentCheck)
                               (let ( (result (get-seme-nodes (car parentCheck)))
                                    )
                                    (set! word-node-seme-nodes-cache
                                         (append word-node-seme-nodes-cache 
                                         (list (cons wordNode result)))
                                    )
                                    result                      
                               )

                              (let ( (partCheck (check-part-whole wordNode nodes))
                                   )
                                   (if (car partCheck)
                                       (let ( (result (get-seme-nodes (car partCheck)))
                                            )
                                            (set! word-node-seme-nodes-cache 
                                                  (append word-node-seme-nodes-cache 
                                                  (list (cons wordNode result)))
                                            )
                                            result
                                       )

                                      (begin
                                          (EvaluationLink (stv 1 1)
                                              (PredicateNode "unknownTerm")
                                              (ListLink
                                                   wordNode
                                              )
                                          )
                                          '()
                                      )
                                   )
                              )
                           ); if
                      ); let
                 ); let
              )
        ); let
    ); if
)

; Called by C++ (updateDialogControllers). Chooses a sentence to say (that has
; already been stored in the AtomSpace by other processing).
(define (choose-sentence)

  (define (get-valid-sentences links)    
    (if (not (null? links))
        (let* ((link (car links))
               (tv (cog-tv link)))
          (if (and (not (null? tv)) (> (assoc-ref (cog-tv->alist tv) 'mean) 0))
              (begin
                (cog-set-tv! link (stv 0 0))
                (fold 
                 (lambda (node result)
                   (append result (list (cog-name node)))
                   )
                 '()
                 (gdr link)
                 )
                )
              (get-valid-sentences (cdr links))
              )
          )
        #f
        )
    )  
  
  (let ((sentences
         (get-valid-sentences
          (cog-get-link
           'ListLink
           'SentenceNode
           (AnchorNode "# Possible Sentences")              
           )
          )
         
         ))
    ; demo rule: return the first sentence
    (if sentences
        (car sentences)
          sentences
          )
    )
  
  )
  
