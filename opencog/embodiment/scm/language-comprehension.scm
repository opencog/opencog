; opencog/embodiment/scm/language-comprehension.scm
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
           (get-seme-nodes wordNode)
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
              (get-seme-nodes groundWN)
              )
             )
           (get-seme-nodes figureWN)
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
           (get-seme-nodes wordNode)
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

; This function check if an ReferenceLink connects a given structure to
; the ConceptNode #you or the agent seme node, determining if the 
; given message was sent or not to the agent
(define (wasAddressedToMe messageTarget )
  (or (equal? messageTarget (ConceptNode "#you"))
      (cog-link 'ReferenceLink messageTarget agentSemeNode ))
)

; This function check if a given PredicateNode belongs to the most recent
; parsed sentence
(define (inLatestSentence predicateNode )
  (if (member predicateNode (get-latest-frame-predicates))
      #t
      #f
      )
  )


(define (createActionCommand framePredicateNode agentNode actionNode arguments )
  (if (and (inLatestSentence framePredicateNode) (wasAddressedToMe agentNode))
      (ExecutionLink (stv 1 1)
       actionNode
       arguments
       )
      '()
      )
)

;;; Helper functions

; Call some method to prepare instance of frames
; given an ungrounded predicateNode
; The given Frame instance can contains VariableNodes
; in its elements values
; If there is a mapped function that will preprocess
; the frame the return value is #t and #f otherwise
(define (frame-preprocessor predicateNode)
  (let ((frameType (get-frame-instance-type predicateNode)))
    (cond ((equal? frameType "#Locative_relation")
           ; first remove all old known spatial relations
           (map
            (lambda (evalLink)              
              (map 
               (lambda (oldFrame)
                 (remove-frame-instance oldFrame)
                 )
               (cog-outgoing-set (car (gdr evalLink)))
              )
              )
             (cog-filter-incoming
              'EvaluationLink
              (PredicateNode "knownSpatialRelations")
              )             
            )

           ; then compute the new spatial relations and re-define the predicate
           (EvaluationLink (stv 1 1)
            (PredicateNode "knownSpatialRelations" )
            (cog-emb-compute-spatial-relations 
             (get-sentence-author (car (get-latest-sentences)))
             (get-grounded-element-value (get-frame-instance-element-value (get-frame-instance-element-predicate predicateNode "Figure")))
             (get-grounded-element-value (get-frame-instance-element-value (get-frame-instance-element-predicate predicateNode "Ground")))
             (get-grounded-element-value (get-frame-instance-element-value (get-frame-instance-element-predicate predicateNode "Ground_2")))
             )
            )

           #t
           )
          )
    #f
    )
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
             ; then disconnect the frame element from the frame instance
             (cog-delete elementLink)
             )
           (cog-filter-incoming
            'FrameElementLink
            predicateNode
            )
           )
         ; finally remove the inheritance link
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
(define (get-latest-sentences)
  (let ((anchors 
         (cog-get-link 
          'ListLink 
          'SentenceNode 
          (AnchorNode "# New Parsed Sentence") 
          ) 
         ) 
        )
    (if (not (null? anchors))        
        (gdr (car anchors) )
        (list)
        )
    )  
)

; Retrieve the node of the agent who says the given sentence
(define (get-sentence-author sentence)
  (let ((author '()))
    (map
     (lambda (link)
       (if (and (equal? (car (gdr link)) sentence) (cog-subtype? 'ObjectNode (cog-type (gar link))))
           (set! author (gar link))
           )
       )    
     (cog-filter-incoming
      'ListLink
      sentence
      )
     )      
    author
    )
  )

; Retrieve a list containing the parses that belongs to the most
; recent parsed sentences
(define (get-latest-parses)
  (let ((parses (list)))
    (map
     (lambda (sentence)
       (map 
        (lambda (parse)          
          (set! parses (append parses (list (gar parse) )))
          )
        (cog-get-link
         'ParseLink
         'ParseNode
         sentence
         )
        )
       )
     (get-latest-sentences)
     )
    parses
    )
)

; Retrieve a list containing the WordInstanceNodes that belongs to the most
; recent sentences parses
(define (get-latest-word-instance-nodes . parses)
  (let ((wins (list)))
    (map
     (lambda (win)
       (map
        (lambda (refLink)
          (set! wins (append wins (cog-outgoing-set (car (gdr refLink)) ) ) )
          )
        (cog-get-link
         'ReferenceLink
         'ListLink
         win
         )
        )
       )
     (if (= (length parses) 0) 
         (get-latest-parses) 
         (if (list? (car parses))
             (car parses)
             parses
             ) ; if
         ) ; if     
     ) ; map
    wins
    ) ; let
  )


; Retrieve a list containing all the PredicateNodes, that represent
; Frames, which belong to the most recent parsed sentences
; It used the most recent WordInstanceNodes to find the PredicateNodes
(define (get-latest-frame-predicates . wordInstanceNodes)
  (let ((predicates (list)))
    (map
     (lambda (win)
       (map
        (lambda (evalLink)
          (let ((elemPredicate  (gar evalLink) ))
            (if (not (null? 
                      (cog-get-link
                       'InheritanceLink
                       'DefinedFrameElementNode
                       elemPredicate
                       )))
                (let ((elemLinks (cog-get-link
                                  'FrameElementLink
                                  'PredicateNode
                                  elemPredicate)))

                  (if (and (not (null? elemLinks))
                           (gar (car elemLinks))
                           (not (null? (cog-get-link
                                        'InheritanceLink
                                        'DefinedFrameNode
                                        (gar (car elemLinks))))))
                      (set! predicates (append predicates (list (gar (car elemLinks))) ))
                      )
                  )
                )
            )
          )
        (cog-get-link
         'EvaluationLink
         'PredicateNode
         win
         )
        )
       )
     (if (= (length wordInstanceNodes) 0) 
         (get-latest-word-instance-nodes) 
         (if (list? (car wordInstanceNodes))
             (car wordInstanceNodes)
             wordInstanceNodes
             )
         )
     )
    (delete-duplicates predicates)
    )
)

; Retrieve all the anaphoric suggestions for a given WordInstanceNode
; that belong to the most recent parsed sentence 
(define (get-anaphoric-suggestions wordInstanceNode)
  ; EvaluationLink
  ;    ConceptNode "anaphoric reference"
  ;    ListLink
  ;       WordInstanceNode <- pronoun
  ;       WordInstanceNode <- suggestion
  (let ((suggestions '()))
    (map
     (lambda (suggestion)
       (let (( pair (car (gdr suggestion ) ) )
             ( strength (assoc-ref (cog-tv->alist (cog-tv suggestion)) 'confidence ) )
             )
         (if (equal? (gar pair) wordInstanceNode)
             (set! suggestions (append suggestions (list (cons (car (gdr pair) ) strength)) ) )
             )
         )
       )
     (cog-get-link
      'EvaluationLink
      'ListLink
      (ConceptNode "anaphoric reference")
      )
     )
    suggestions
    )
  
)

; Retrieve the WordNode related to a given WordInstanceNode
(define (get-word-node wordInstanceNode)
  ; ReferenceLink
  ;    WordInstanceNode
  ;    WordNode
  (let ((wordNodes (cog-get-link
                    'ReferenceLink
                    'WordNode
                    wordInstanceNode
                    )
                   ))
    (if (not (null? wordNodes))
          (car (gdr (car wordNodes)))
          '()
          )    
    )
  )


; SemeNodes are described by WordNodes
; Each WordNode can have many SemeNodes attached
; to it by a ReferenceLink. This function retrieves
; all the SemeNodes attached to a given WordNode
(define (get-seme-nodes wordNode)
  ; ReferenceLink
  ;    SemeNode
  ;    WordNode  
  (let ((validSemeNodes (list)))
    (map
     (lambda (refLink)
       (set! validSemeNodes (append validSemeNodes (list (gar refLink) ) ) )
       )     
     (cog-get-link
      'ReferenceLink
      'SemeNode
      wordNode
      )
     )
    validSemeNodes
    )
  )

; Each SemeNode is connected to a node that represents
; a real object into the Environment. So this function
; returns that node using the given SemeNode for that.
(define (get-real-node semeNode)
   ; ReferenceLink
   ;    ObjectNode (or a child of it)
   ;    SemeNode  
  (let ((realObject '()))
    (map
     (lambda (candidateLink)
       (let ((object (cog-get-partner candidateLink semeNode)))
         (if (cog-subtype? 'ObjectNode (cog-type object) )
             (set! realObject object)
             )
         )
       )
     (cog-filter 
      'ReferenceLink 
      (cog-incoming-set semeNode)
      )
     )
    realObject
    )  
  )

; Given a WordInstanceNode, this function will try
; to find all the Corresponding SemeNodes.
; SemeNodes can be grounded by WordInstanceNodes of
; type nouns and pronouns, so only this two types
; of WordInstanceNodes will return candidate SemeNodes
; SemeNodes are connected to WordNodes, so getting 
; the WordNode of the WordInstanceNode it is possible
; to retrieve its SemeNodes. If the WordInstanceNode was a pronoun
; the anaphoric suggestions will be used to build the SemeNodes list
; Each WordInstanceNode, suggested in the anaphora, will have its 
; WordNode evaluated and, consequently, all the SemeNodes related
; to these WordNodes will become part of the final list
; The final list contains not only the SemeNodes but its strengths
; i.e. ( ( (SemeNode "1") . 0.03)
;        ( (SemeNode "2") . 0)
;        ( (SemeNode "3") . 1.) )
(define (get-candidates-seme-nodes wordInstanceNode)
  (let ((wordInstanceNodes '())
        (semeNodes '()))
    
    (cond ( (not (null?
                  (cog-link 
                   'InheritanceLink
                   wordInstanceNode
                   (DefinedLinguisticConceptNode "pronoun")           
                   ) ) )
                   ; look for anaphoric reference
            (let ((anaphoricSuggestions (get-anaphoric-suggestions wordInstanceNode) ))
              (if (not (null? anaphoricSuggestions ) )
                  (map
                   (lambda (suggestedWin)
                     (set! wordInstanceNodes (append wordInstanceNodes (list suggestedWin ) ) )
                     )
                   anaphoricSuggestions
                   )
                  (set! wordInstanceNodes (append wordInstanceNodes (list (cons wordInstanceNode 0 ) ) ) )
                  )
              ) ; let          
            )
          ( (not (null? 
                  (cog-link 
                   'PartOfSpeechLink ; else if
                   wordInstanceNode
                   (DefinedLinguisticConceptNode "noun" )
                   ) ) )
            (set! wordInstanceNodes (append wordInstanceNodes (list (cons wordInstanceNode 0 ) ) ) )
            )
          ); cond

    (map
     (lambda (candidate)
       (let* (
              (noun (car candidate))
              (strength (cdr candidate))
              (groundedSemeNode (cog-get-link 'ReferenceLink 'SemeNode noun ))
              )
         (if (not (null? groundedSemeNode))
             (set! semeNodes (append semeNodes (list (cons strength (list (gar (car groundedSemeNode)) ) ) )))
             (let ((wordNode (get-word-node noun ) ) ) ; else
               (if (not (null? wordNode))
                   (set! semeNodes (append semeNodes (list (cons strength (get-seme-nodes wordNode)) ) ))
                   ) ; if
               ); let
             ) ; if
         ) ; let
       )
     wordInstanceNodes
     ) ; map

    semeNodes
    ); let
)


; Remove duplicated elements from list
(define (unique-list ls)
  (if (list? ls)
      (let ((finalList '()))

        (map
         (lambda (element)
           (if (not (member element finalList))
               (set! finalList (append finalList (list element)))
               )
           )
         ls
         )
        finalList
        )
      ls
      )
  )

; Given a list of candidates in the format:
; ( (WordInstanceNode1 SemeNode1, ..., SemeNodeN)
;   (WordInstanceNode2 SemeNode1, ..., SemeNodeN)
;   (WordInstanceNodeM SemeNode1, ..., SemeNodeN) )
; and a list of strengths:
; ( (SemeNode1 0)
;   (SemeNode2 0.3)
;   (SemeNodeM 1.0) )
;
; this function tries to keep just on WordInstanceNode and a corresponding SemeNode.
; If there is one SemeNode which has a greater strength than others it will happen.
; However, a list containing the SemeNodes with greater strengths for each
; WordInstanceNode will be returned
(define (filter-by-strength candidates strengths)
  (let (( filtered '() ))
    (map
     (lambda (candidate)
       (let ((key (car candidate))
             (values (cdr candidate))
             (selectedSemeNodes '())
             (strongest 0)
             )
         
         (map
          (lambda (semeNode)
            (cond ((> (assoc-ref strengths semeNode) strongest)
                   (set! selectedSemeNodes (list semeNode ) )
                   (set! strongest (assoc-ref strengths semeNode))
                   )

                  ((= (assoc-ref strengths semeNode) strongest)
                   (set! selectedSemeNodes (append selectedSemeNodes (list semeNode)))
                   )

                  )
            )
          values
          )

         (if (null? selectedSemeNodes)
             (set! filtered candidate)
             (set! filtered (append filtered (list (cons key selectedSemeNodes ) ) ) )
             )

         ) ; let
       ) ; lambda
     candidates
     )
    filtered
    )
)

; Given a list of objects in the format:
; ( (WordInstanceNode1 SemeNode1, ..., SemeNodeN )
;   (WordInstanceNode2 SemeNode1, ..., SemeNodeN )
;   (WordInstanceNode2 SemeNode1, ..., SemeNodeN ) )
; and a list of SemeNodes that will be used
; to filter the objects lists. Each list of the objects list
; must contains only the semeNodes listed in the SemeNodes list
(define (filter-objects objects semeNodes)
  (let* ((key (car semeNodes))
         (values (cdr semeNodes))
         (newValues '())
         (filterObjects (assoc-ref objects key))
         )
    (if filterObjects
        (set! newValues (filter (lambda (x) (member x values)) filterObjects ))
        )

    (if (not (null? newValues))
        (set! newValues (append (delete (assoc key objects) objects ) (list (cons key newValues) )))
        (set! newValues (delete (assoc key objects) objects ))
        )
    newValues
    )
  )

; Given a list of semeNodes, the nearest to the current agent
; position will be considered the nearest SemeNode.
; To determine the distance between the agent and the SemeNode
; (which represents a real element into the environment) 
; The TruthValue of the predicate "proximity" is evaluated 
(define (get-nearest-candidate semeNodes)
  (let ((agentNode (get-real-node agentSemeNode))
        (candidates '())
        )   
    (map
     (lambda (candidate)
       (let ((distance (assoc-ref 
                        (cog-tv->alist 
                         (cog-tv 
                          (EvaluationLink
                           (PredicateNode "proximity")
                           (ListLink
                            agentNode
                            (get-real-node candidate)
                            )
                           )
                          )
                         ) 'mean ) ))

         (if (or (= (length candidates) 0) (> distance  (cdr (car candidates))))
             (set! candidates (list (cons candidate distance)))
             (if (and (> (length candidates) 0) (= distance  (cdr (car candidates))))
                 (set! candidates (append candidates (list (cons candidate candidates))))
                 )
             )         
       ) ; let       
     ) ; lambda
     semeNodes
    ) ; map
    
    (if (> (length candidates) 0)
        (car (car candidates))
        '()
        )
    ) ; let
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



; Given a PredicateNode, which representes a 
; Frame element that belongs to a Frame instance,
; this function returns the Element type in a string format
; or null if the given argument isn't a Frame Element Predicate
; i.e. (get-frame-instance-element-type 
;        (PredicateNode "ball_99_color_Color") ) = "#Color:Color"
(define (get-frame-instance-element-type predicateNode)
  (let ((link (cog-get-link
               'InheritanceLink
               'DefinedFrameElementNode
               predicateNode
               )))
    (if (not (null? link))
        (cog-name (car (gdr (car link)) ))
        '()
        )
    )    
  )

; Given a PredicateNode, which representes a 
; Frame instance, this function returns the Frame type
; in a string format or null if the given argument 
; isn't a Frame Instance Predicate
; i.e. (get-frame-instance-type
;        (PredicateNode "ball_99_color") ) = "#Color"
(define (get-frame-instance-type predicateNode)
  (let ((link (cog-get-link
               'InheritanceLink
               'DefinedFrameNode
               predicateNode
               )))
    (if (not (null? link))
        (cog-name (car (gdr (car link)) ))
        '()
        )
    )
  )

; Given a PredicateNode, which representes a
; Frame instance, this function returns a list containing
; the predicates that represents each element of the 
; Frame Instance
; i.e. (get-frame-instance-elements-predicates
;        (PredicateNode "ball_99_color") ) = (
;           (PredicateNode "ball_99_color_Entity")
;           (PredicateNode "ball_99_color_Color")
;      )
(define (get-frame-instance-elements-predicates predicateNode)
  (let ((predicates '()))
    (map
     (lambda (link)
       (let ((candidate (car (gdr link))))
         (if (and (equal? 'PredicateNode (cog-type candidate)) (not (equal? predicateNode candidate)))
             (set! predicates (append predicates (list candidate)))
             )
         )
       )
     (cog-filter-incoming
      'FrameElementLink
      predicateNode
      )
     )
    predicates
    )
  )

; Given a PredicateNode, which representes a
; Frame element that belongs to a Frame instance,
; this function returns the Element valu
; i.e. (get-frame-instance-element-value
;        (PredicateNode "ball_99_color_Color") ) = (ConceptNode "blue")
(define (get-frame-instance-element-value elementPredicateNode)
  (if (not (null? elementPredicateNode))
      (let ((values
             (cog-filter-incoming
              'EvaluationLink
              elementPredicateNode
              ))
            )
        (if values
            (car (gdr (car values)))
            '()
            )
        )
      '()
      )
  )

; Given a PredicateNode, which represents a
; Frame instance and a specific element name (second part of 
; the element type, after the colon i.e. #Color:Entity ; element name = Entity)
; in a string format, this function will return the PredicateNode which
; represents the desired Frame Element
; i.e. (get-frame-instance-element-predicate
;        (PredicateNode "ball_99_color") "Entity" ) = (PredicateNode "ball_99_Entity")
(define (get-frame-instance-element-predicate predicateNode elementName)  
  (let ((elementPredicateNode '()))
    (map
     (lambda (elementPredicate)
       (let* (
	      (name (get-frame-instance-element-type elementPredicate))
	      (colonIndex (list-index (lambda (char) (char=? char #\:)) (string->list name)))
	      )
	 ; do a split in the element name and compare it without the frame prefix
	 ; i.e. it name is #Color:Entity, but compares only the second part Entity == elementName
	 (if (and colonIndex (string=? (substring name (+ colonIndex 1) (string-length name)) elementName) )
	     (set! elementPredicateNode elementPredicate)
	     )
	 )
       )
     (get-frame-instance-elements-predicates predicateNode)
     )
    elementPredicateNode
    )
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

; Given a specific Frame type (string format) and a
; given element name (string format), this function returns
; the DefinedFrameElementNode which represents the desired element,
; if it was a valid element and null otherwise.
; i.e. (get-frame-element-node "#Color" "Entity") = 
;         (DefinedFrameElementNode "#Color:Entity")
;
;      (get-frame-element-node "#Color" "Sunda") = '() ; there
(define (get-frame-element-node frameType elementName)
  (let ((chosenElementNode '()))
    (map
     (lambda (elementNode)
       (let* ((name (cog-name elementNode))
              (colonIndex (list-index (lambda (char) (char=? char #\:)) (string->list name)))
              )
         ; do a split in the element name and compare it without the frame prefix
         ; i.e. it name is #Color:Entity, but compares only the second part Entity == elementName
         (if (and colonIndex (string=? (substring name (+ colonIndex 1) (string-length name)) elementName) )
             (set! chosenElementNode elementNode)
             )
         )
       )
     (get-frame-elements frameType)
     )
    chosenElementNode
    )
  )


; Given the type of a valid Frame, this function will returns
; a list containing all the elements of the Frame, represented by
; DefinedFrameElementNodes.
; i.e. (get-frame-elements "#Color") = (
;         (DefinedFrameElementNode "#Color:Entity")
;         (DefinedFrameElementNode "#Color:Color")
;      )
(define (get-frame-elements frameType)
  (let ((elements '())
        (frameNode (DefinedFrameNode frameType))
        )
    (map
     (lambda (link)
       (set! elements (append elements (gdr link)))
       )
     (cog-get-link
      'FrameElementLink
      'DefinedFrameElementNode
      frameNode
      )    
     )
    
    (map
     (lambda (link)
       (let ((parent (car (gdr link))))
         (if (and (equal? 'DefinedFrameNode (cog-type parent)) (not (equal? parent frameNode)) )
             (set! elements (append elements (get-frame-elements (cog-name parent))))
             )
         )
       )
     (cog-filter-incoming
      'InheritanceLink
      frameNode
      )
     )

    elements
    )
  )

; Nodes of several types can be part of structures
; which has a semantic value (i.e. a sentence said by an avatar, 
; composed by several frames).
; However, the agent's perceptions structures, stored into the 
; AtomSpace, uses a few types of nodes to keep 
; the maintenance of the knowledge database easier as possible.
; So, this function normalizes a node given as argument
; to make it useful in the PatternMatching process.
; i.e. (get-corresponding-node (WordNode "ball") ) = (ConceptNode "ball")
;      (get-corresponding-node (WordNode "hungry") ) = (ConceptNode "hunger")
(define (get-corresponding-node word)
  (let ((node (assoc-ref (get-word-node-map) word)))
    
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


; A Frame instance can be prepared to be used in a Pattern Matching
; operation. Suppose we know the value of one element of a specific Frame instance
; which has two elements and we want to know the value of the other element.
; So, we could prepare a Frame Instance with a VariableNode in the value of the desired
; element and do a PatternMatching. This function tells us if a given PredicateNode
; which represents a Frame Instance has VariableNodes in its values.
; i.e. (frame-instance-contains-variable? (PredicateNode "ball_99_color")) = #f
(define (frame-instance-contains-variable? predicateNode)
  (let ((containsVariable? #f))
    (map
     (lambda (predicate)
       (let ((value (get-frame-instance-element-value predicate)))
         (if (and (not (null? value)) (equal? 'VariableNode (cog-type value)))
             (set! containsVariable? #t)
             )
         )
       )
     (get-frame-instance-elements-predicates predicateNode)
     )    
    containsVariable?
    )
  )

; Check if a given Frame instance, represented by the given PredicateNode,
; has a correspondent instance which has SemeNodes and ConceptNodes in its
; elements, what means that there is a "generated by the agent" Frame 
; instance into the AtomTable
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
        (elementsPredicates (get-frame-instance-elements-predicates predicateNode))
        (numberOfElements (length elementsPredicates))
        (candidates '())
        (chosenPredicate '())
        )
    
    (map
     (lambda (elementPredicate)
       (let* ((elementValue (get-frame-instance-element-value elementPredicate))
              (elementType (get-frame-instance-element-type elementPredicate))
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


; Given a specific node as value, this function,
; will try to find a node wich grounds the given one.
; For instance, a grounded node for a SemeNode is a ObjectNode (or a child of it)
; i.e. (get-grounded-element-value (WordNode "blue")) = (ConceptNode "blue")
(define (get-grounded-element-value value)
  (if (or (null? value) (equal? (cog-type value) 'VariableNode))
      value
      (let ((groundedValue '()))
      ; first try to find a reference resolution semeNode
        (if (equal? (cog-type value) 'WordInstanceNode)
            (let ((refLinks (cog-get-link
                             'ReferenceLink
                             'SemeNode
                             value)
                            ))
              (if (not (null? refLinks))
                  (set! groundedValue (gar (car refLinks)))
                  ) ; let             
              ) ; if
            ) ; if
        
       ; if there is no semeNode, then try to find a corresponding node
        (if (null? groundedValue)
            (let ((name (cog-name value)))
              (if (and (> (string-length name) 1) (string=? "#" (substring name 0 1)))
                  (set! groundedValue (get-corresponding-node (substring name 1 (string-length name))))
                  (let ((wordNode (get-word-node value)))
                    (if (not (null? wordNode))
                        (set! groundedValue (get-corresponding-node (cog-name wordNode)))
                        )
                    )
                  )
              )
            ) ; if
        groundedValue
        ) ; let  
      )
  )


; Given a predicate which represents a frame instance
; this function tries to find a grounded frame that
; matches the given frame
(define (match-frame predicateNode)
 
  (let ((candidates '())
        (evaluatedElements 0)
        (frameType (get-frame-instance-type predicateNode))
        (elementsStrength '())
        )

    (map ; inspect all the frame elements
     (lambda (elementPredicate)
       (let* (
              (value (get-frame-instance-element-value elementPredicate))
              (groundedValue (get-grounded-element-value value))
              (elementType (get-frame-instance-element-type elementPredicate))
              (elementName (get-frame-element-name elementType))
              (elementTypeNode (get-frame-element-node frameType elementName ))
              (elementsCandidates '())
             )
         ; check if the element has a Variable as its value
         (if (equal? 'VariableNode (cog-type value))
             ; ok, it has a variable and must be matched against a stored frame
             (map  ; retrieve from AT all elements of the same type
              (lambda (inheritanceLink)
                (let* ((candidate (gar inheritanceLink ))
                       (frameInstancePredicate (gar (car (cog-filter-incoming 'FrameElementLink candidate))))
                       )
                  ; only PredicateNodes are welcome
                  (if (equal? 'PredicateNode (cog-type candidate))
                      
                      (map
                       (lambda (evalLink)
                         (let* ((elementValue (car (gdr evalLink)))
                                (valueType (cog-type elementValue))
                                )
                           (if (or (equal? 'SemeNode valueType) (equal? 'ConceptNode valueType))                               
                               (set! elementsCandidates (append elementsCandidates (list frameInstancePredicate)))
                               (set! elementsStrength (append elementsStrength (list (cons frameInstancePredicate 
                                   (assoc-ref (cog-tv->alist (cog-tv inheritanceLink)) 'mean)))))
                               )
                           )
                         )
                       (cog-filter-incoming 'EvaluationLink candidate)
                       )
                      )
                  )
                )
              (cog-filter-incoming 'InheritanceLink elementTypeNode)
              )
             (map ; else
              (lambda (evalLink)
                (let* ((candidate (gar evalLink))
                       (candidateType (cog-type candidate))
                      )
                  (if (and (equal? 'PredicateNode candidateType) 
                           (not (null? (cog-link 'InheritanceLink candidate elementTypeNode))) )
                      (let ((frameInstancePredicate (gar (car (cog-filter-incoming 'FrameElementLink candidate)))))
                        (if (not (equal? frameInstancePredicate predicateNode))
                    	    (begin
                        	(set! elementsCandidates (append elementsCandidates (list frameInstancePredicate)))
                        	(set! elementsStrength (append elementsStrength (list (cons frameInstancePredicate 
                            	     (assoc-ref (cog-tv->alist (cog-tv (cog-link 'InheritanceLink candidate elementTypeNode))) 'mean)))))
                             )
                            )
                        )
                      )
                  )
                )
              (cog-filter-incoming 'EvaluationLink groundedValue)
              )
             ) ; if

         (if (= evaluatedElements 0)
             (set! candidates elementsCandidates)
             (set! candidates (filter (lambda (x) (member x elementsCandidates)) candidates))
             )
         )
       
       (set! evaluatedElements 1)
       )
     (get-frame-instance-elements-predicates predicateNode)
     )
    ; sort by the candidate tv mean strength. strongest first.
    (sort candidates (lambda (x y) (> (assoc-ref elementsStrength x) (assoc-ref elementsStrength y))) )
    )
  )

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
       (let* ((value (get-frame-instance-element-value predicate))
	      (groundedValue (get-grounded-element-value value))
              (variable? (and value (equal? 'VariableNode (cog-type value))))
              (elementNode (get-frame-element-node frameType (get-frame-element-name (get-frame-instance-element-type predicate) )))
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
     (get-frame-instance-elements-predicates predicateNode)
     )

    (VariableScopeLink
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


; This function receives as argument a list of PredicateNodes that
; represents instances of Frames and returns a list of PredicateNodes
; that is grounded versions of the given ones
(define (find-grounded-frame-instances-predicates . framesPredicates)
  (let ((finalFrames '()))
    (map ; ok it is a question, so handle it
	; start by creating new frames with SemeNodes as element values instead of nouns/pronouns WINs
	  ;questionFrames
     (lambda (predicate)
       (frame-preprocessor predicate)
       (if (not (frame-instance-contains-variable? predicate))
           (let ((groundedFrameInstance (get-grounded-frame-instance-predicate predicate)))
             (if (not (null? groundedFrameInstance))                                    
                 (set! finalFrames (append finalFrames (list groundedFrameInstance )))                   
                 )
             ) ; let
           (let ((groundedFrameInstances (match-frame predicate)))
             (if (not (null? groundedFrameInstances))
                 (set! finalFrames (append finalFrames (list (car groundedFrameInstances ))))
                 )
             ) ; let
           ) ; if
       ) ; lambda
     (if (list? (car framesPredicates)) (car framesPredicates) framesPredicates) 
     )
    finalFrames
    )
  )


;;; Core functions

; This function execute the whole process of Reference resolution
; It must be called after a new sentence has been loaded into the AtomTable.
; Reference resolution is a process that uses perceptions and predicates,
; built using the state of the Environment which contains the agent, to identify
; the real elements that were mentioned by someone in a given sentence
; The output of this function is a list of WordInstanceNodes (nouns and pronouns)
; and one SemeNode for each WordInstanceNode that was chosen by the Reference resolution
; process
(define (resolve-reference)
  (let ((objects '())
        (resolvedReferences '())
        (anaphoricSemeNodeStrength '())
        (groundedRulesCounter '())
        )    
    ; first retrieve all objects, from the latest sentence, to be evaluated
    (map 
     (lambda (win)

       (let ((semeNodes '())
             (semeNodesCandidates (get-candidates-seme-nodes win)))

         (map
          (lambda (candidateSemeNodesList)          
            (let ((strength (car candidateSemeNodesList))
                  (values (cdr candidateSemeNodesList)))
              (map
               (lambda (semeNode)
                 (set! semeNodes (append semeNodes (list semeNode) ) )
                 ; keep the greater strength suggestion
                 (let ((oldSuggestion (assoc semeNode anaphoricSemeNodeStrength)))
                   (cond ( (and 
                            oldSuggestion
                            (> strength (cdr oldSuggestion) ) 
                            )
                           (set! anaphoricSemeNodeStrength (alist-delete semeNode anaphoricSemeNodeStrength ) )
                           (set! oldSuggestion #f)
                           ))

                   (if (not oldSuggestion)
                       (set! anaphoricSemeNodeStrength (append anaphoricSemeNodeStrength (list (cons semeNode strength ) ) ) )
                       )
                   
                   ) ; let                 
                 (set! groundedRulesCounter (append groundedRulesCounter (list (cons semeNode 0 ) ) ) )
                 ) ; lambda
               values
               ) ; map
              ) ; let
            ) ; lambda
          semeNodesCandidates
          ) ; map

         (if (not (null? semeNodes))
             (set! objects (append objects (list (cons win (delete-duplicates semeNodes )))))
             )

         ) ; let
       ) ; lambda
     (get-latest-word-instance-nodes)
     )

    ; now use the rules to filter the objects list
    (map
     (lambda (rule)
       (map
        (lambda (winAndSemesListLink)
          (cond ((not (null? winAndSemesListLink))
                 (let* ((winAndSemes (cog-outgoing-set winAndSemesListLink))
                        (win (car winAndSemes))
                        (semes (cdr winAndSemes))
                        )

                        (map
                         (lambda (semeNode)
                           (let ((rulesCounter (+ (assoc-ref groundedRulesCounter semeNode) 1)) )
                             (set! groundedRulesCounter (alist-delete semeNode groundedRulesCounter))
                             (set! groundedRulesCounter (alist-cons semeNode rulesCounter groundedRulesCounter))
                             )
                           )
                         semes
                         )
                        ; remove those semeNodes that must not be present in the answer
                        (set! objects (filter-objects objects winAndSemes) )
                        ) ; let*
                 )) ; cond
          )
        (cog-outgoing-set (do-varscope rule ) )
        )

       )
     reference-resolution-rules
     )


    ; filter by the number of satisfied rules
    (set! objects (filter-by-strength objects groundedRulesCounter ) )

    ; filter the objects by their strengths given by the anaphora resolution
    ; if there is no anaphoric suggestion, the objects list will remains the same
    (set! objects (filter-by-strength objects anaphoricSemeNodeStrength ) )
    
    ; now apply the latest filter, the distance
    (map
     (lambda (instance)
       (let ((semeNode (get-nearest-candidate (cdr instance) ))
             (win (car instance))             
             )
         (set! resolvedReferences (append resolvedReferences (list (ReferenceLink (stv 1 1) semeNode win))))
         )       
       )
     objects
      )
    
    resolvedReferences
    
    )
  
)

; When a sentence containing an imperative verb is parsed
; Frames that represents the given command can be identified and then
; transformed into a grounded command. This function is responsible
; to to that work. It identifies the presence of Frames instance
; in the most recent parsed sentence and tries to recognize given commands
(define (resolve-command)
  (let ((commands '() ))
    ; set the truth value of the latests eval links to false
    (map
     (lambda (evalLink)
       (cog-set-tv! evalLink (stv 0 0))
       )
     (cog-get-link 'EvaluationLink 'ListLink (PredicateNode "latestAvatarRequestedCommands"))
     )
    (map
     (lambda (rule)

       (map
        (lambda (candidateCommand)
          (if (not (member candidateCommand commands))
              (set! commands (append commands (list candidateCommand )) )
              )
          )
        (cog-outgoing-set (do-varscope rule ) )
        )
              
       )
     command-resolution-rules
     )

    (cond ((not (null? commands))           
           (EvaluationLink 
            (stv 1 1)
            (PredicateNode "latestAvatarRequestedCommands")
            (ListLink
             (unique-list commands)
             )
            )

           )) ; cond
    commands
    ) ; let
  )


; This function is responsible for getting a bunch of Frames instances
; that represents a question made by another agent and start a pattern
; matching process to find the answer inside the agent's AtomSpace.
; The matching starts by getting all parses of the sentence that 
; represent the questions. Each parse has a list of frames.
; The parse which matches a greater number of frames will be chosen
; as the question answer. 
(define (answer-question)
  (let ((chosenAnswer '())
        (question? #f)
        (questionType '())
        )
    (map
     (lambda (parse)
       (let ((incomingPredicates (get-latest-frame-predicates (get-latest-word-instance-nodes parse)))
             (questionParse? #f)
             (questionFrames '())
             )

         (map
          (lambda (predicate)
            (if (string=? (get-frame-instance-type predicate) "#Questioning")
                (begin
                  (set! question? #t)
                  (set! questionParse? #t)
                  (set! questionType
                        (cog-name
                         (get-frame-instance-element-value
                          (get-frame-instance-element-predicate predicate "Manner")
                          )
                         )
                        )
                  )
                (if (not (member (get-frame-instance-type predicate) invalid-question-frames))
                    (set! questionFrames (append questionFrames (list predicate)))
                    )
                )
            )
          incomingPredicates
          )

         (if questionParse?
             (let* ((numberOfIncomingPredicates (length questionFrames))
                    (groundedPredicates (find-grounded-frame-instances-predicates questionFrames))
                    (numberOfGroundedPredicates (length groundedPredicates))
                    (balance (- numberOfIncomingPredicates numberOfGroundedPredicates))
                    )
               (if (and (> numberOfIncomingPredicates 0) (= balance 0) (or (null? chosenAnswer) (> numberOfGroundedPredicates (car chosenAnswer))))
                   (set! chosenAnswer (cons numberOfGroundedPredicates groundedPredicates))
                   )
               )
             )
         )
       )
     (get-latest-parses)
     )

     (if question?
         (begin
          ; first remove old predicates
          (map
           (lambda (evalLink)
             (cog-set-tv! evalLink (stv 0 0))
             )
           (cog-get-link 'EvaluationLink 'ListLink (PredicateNode "latestQuestionFrames"))
           )
          ; then create a new one
          (EvaluationLink (stv 1 1)
             (PredicateNode "latestQuestionFrames")
             (ListLink
              (if (not (null? chosenAnswer) ) (cdr chosenAnswer) '() )
              )
             )

          questionType
          )
        '()
        ) ; if
  
     ) ; let
  )

