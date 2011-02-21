; opencog/embodiment/scm/question-answering.scm
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

; JaredW: TODO This seems to be a weird temporary hack. I've re-enabled the Attributes Frame so it can handle "is" sentences. 
(define invalid-question-frames
  (list
   "#Attributes"
   "#Possibilities"
   "#Temporal_colocation"
   "#Entity"
   "#Intentionally_act"
   "#Transitive_action"
   "#Excreting"
   "#Desiring"
   "#Successful_action"
   "#Request"
   "#Purpose"
   "#Ingest_substance"
   "#Existence"
   "#Inheritance"
   "#Categorization"
   "#Part_whole"
   "#Physical_entity"
   "#Taking_sides"
   "#Position_on_a_scale"
   "#Assessing"
   "#Morality_evaluation"
   "#Make_noise"
   )
  )

(define normalized-names
  (list 

   (list "happiness" "happy" "happily")
   (list "fear" "fright" "panic" "fearful" "fearfulness" "scared")
   (list "pride" "proud" "pridefulness")
   (list "love" "loving" "loves" "loved" "lovely")
   (list "hate" "hated" "hates" "hating" "hatred" "dislike" "hateful" "hatefully" "hatefulness")
   (list "anger" "angry" "angered" "angering" "angers" "rage" "fury" "wrath" "ire" "choler")
   (list "gratitude" "grateful" "gratefulness" "thankfulness")
   (list "excitement" "excited" "exhilaration")
          
   (list "hunger" "hungered" "hungering" "hungers" "hungry" "hungriness" "hungrily")
   (list "thirst" "thirsted" "thirsts" "thirsty" "thirstiness" "thirsting")
   (list "poo" "excrement" "poo_urgency" "pooed" "pooing" "poos" "defecating" "defecated" "defecates")
   (list "pee" "urine" "pee_urgency" "pees" "peeing" "peed" "urinating" "urinated" "urinates")
   (list "fitness" "fittingness")
   (list "energy" "vigor" "power" "vitality" "vigour" "powered" "energized")
   
   ; TODO Fabricio's Relex2Frame modifications break stemming for nouns so you have to do this.
   (list "kicking" "kick" "kicked")
   (list "human" "humans" "man" "men")
   (list "cat" "cats" "Cats")
   (list "mouse" "mice" "Mice")
   (list "mammal" "mammals" "Mammals")
   (list "man" "men" "Men")
   (list "human" "humans" "Humans")
   )
  )

(define word-node-map '())

(define (get-word-node-map)
  (if (null? word-node-map)
      (set! word-node-map
            (append (let ((elements '()))
                      (map
                       (lambda (words)
                         (map
                          (lambda (word)
                            (set! elements (append elements (list (cons word (ConceptNode (car words)) ))))
                            )
                          (cdr words)
                          )
                         (set! elements (append elements (list (cons (car words) (ConceptNode (car words)) ))))
                         )
                       normalized-names
                       )
                      elements
                      )                   
                    (list (cons "you" agentSemeNode))
                    )
            )
      )
  word-node-map
  )
