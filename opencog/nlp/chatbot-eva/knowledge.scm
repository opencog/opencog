;
; knowledge.scm
;
; Knowledge representation of grounded word meanings.
;
; This file encodes associations between words, 

;--------------------------------------------------------------------
; Global knowledge about spatial directions.  The coordinate system
; is specific to the HR robot head.  Distance in meters, the origin
; of the system is behind the eyes, middle of head.  "forward" is the
; object the chest is facing.
(DefineLink
	(DefinedSchema "rightwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number -0.5) ; y is right
		(Number 0)    ; z is up
	))

(DefineLink
	(DefinedSchema "leftwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number 0.5)  ; y is right
		(Number 0)    ; z is up
	))

(DefineLink
	(DefinedSchema "upwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number 0)    ; y is right
		(Number 0.3)  ; z is up
	))

(DefineLink
	(DefinedSchema "downwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number 0)    ; y is right
		(Number -0.3) ; z is up
	))

(DefineLink
	(DefinedSchema "forwards")
	(ListLink ;; three numbers: x,y,z
		(Number 1)    ; x is forward
		(Number 0) ; y is right
		(Number 0)    ; z is up
	))

;--------------------------------------------------------------------
; Global knowledge about word-meaning.
; Specific words have very concrete associations with physical objects
; or actions.

; Knowledge about spatial directions. Pair up words and physical
; directions.
(ReferenceLink (WordNode "up")       (DefinedSchema "upwards"))
(ReferenceLink (WordNode "down")     (DefinedSchema "downwards"))
(ReferenceLink (WordNode "right")    (DefinedSchema "rightwards"))
(ReferenceLink (WordNode "left")     (DefinedSchema "leftwards"))
(ReferenceLink (WordNode "forward")  (DefinedSchema "forwards"))
(ReferenceLink (WordNode "ahead")    (DefinedSchema "forwards"))

; Syntactic category of schema. Used for contextual understanding.
(InheritanceLink (DefinedSchema "upwards")    (ConceptNode "schema-direction"))
(InheritanceLink (DefinedSchema "downwards")  (ConceptNode "schema-direction"))
(InheritanceLink (DefinedSchema "rightwards") (ConceptNode "schema-direction"))
(InheritanceLink (DefinedSchema "leftwards")  (ConceptNode "schema-direction"))
(InheritanceLink (DefinedSchema "forwards")   (ConceptNode "schema-direction"))

; Physical (motor control) knowledge about imperative verbs.
(ReferenceLink (WordNode "look") (GroundedPredicate "py:gaze_at_point"))
(ReferenceLink (WordNode "turn") (GroundedPredicate "py:look_at_point"))

; Syntactic category of imperative verbs.
(InheritanceLink (GroundedPredicate "py:gaze_at_point")
	(ConceptNode "pred-direction"))
(InheritanceLink (GroundedPredicate "py:look_at_point")
	(ConceptNode "pred-direction"))

; Allowed syntactic structure of action-knowledge --
;
; There are several ways to think of this. One way is to think of
; "pred-direction" to be a kind-of subroutine call, whose only valid
; arguments are directions.  Another way of thinking about this is
; as a connector-linkage: "pred-direction" is a connector that can
; only link to the "schema-direction" connector.
;
; This is needed to disambiguate word senses. Consider, for example,
; "look up" and "look sad".  One of these, as a grounded action, can
; only take a direction; the other can only take an expression-name.
; The syntax is used to guarantee that the grounded word-senses are
; compatible.
;
; Specifies the syntactic structure for physical motor-control commands.
(EvaluationLink
	(PredicateNode "turn-action")
	(ListLink
		(ConceptNode "pred-direction")
		(ConceptNode "schema-direction")))

; ---------------------------------------------------------------
; Duplicate of the above, except that this is for use in controlling
; the self-model, rather than the physical motors.  We can't quite
; recycle the above (I tried) because DefinedSchema mis-behaves
; in various irritating ways, so we duplicate the above using
; ConcpetNode, instead.

; Knowledge about spatial directions. Pair up words and physical
; directions.
(ReferenceLink (WordNode "up")       (Concept "upward"))
(ReferenceLink (WordNode "down")     (Concept "downward"))
(ReferenceLink (WordNode "right")    (Concept "rightwards"))
(ReferenceLink (WordNode "left")     (Concept "leftwards"))
(ReferenceLink (WordNode "forward")  (Concept "forward"))
(ReferenceLink (WordNode "ahead")    (Concept "forward"))

; Syntactic category of schema. Used for contextual understanding.
(InheritanceLink (Concept "upward")    (ConceptNode "schema-direction"))
(InheritanceLink (Concept "downward")  (ConceptNode "schema-direction"))
(InheritanceLink (Concept "rightwards") (ConceptNode "schema-direction"))
(InheritanceLink (Concept "leftwards")  (ConceptNode "schema-direction"))
(InheritanceLink (Concept "forward")   (ConceptNode "schema-direction"))

; Model (self-awareness) knowledge about imperative verbs.
(ReferenceLink (WordNode "look") (AnchorNode "*-gaze-direction-*"))
(ReferenceLink (WordNode "turn") (AnchorNode "*-head-direction-*"))

(InheritanceLink (AnchorNode "*-gaze-direction-*")
	(ConceptNode "model-direction"))
(InheritanceLink (AnchorNode "*-head-direction-*")
	(ConceptNode "model-direction"))

; Specifies the syntactic structure for self-model (self-awareness).
(EvaluationLink
	(PredicateNode "turn-model")
	(ListLink
		(ConceptNode "model-direction")
		(ConceptNode "schema-direction")))

;--------------------------------------------------------------------
; Emotional expression semantics (groundings)
(DefineLink
	(DefinedSchema "happy")
	(ListLink
		(Concept "happy")    ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "sad")
	(ListLink
		(Concept "sad")      ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "comprehending")
	(ListLink
		(Concept "comprehending") ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "engaged")
	(ListLink
		(Concept "engaged")  ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "bored")
	(ListLink
		(Concept "bored")    ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "irritated")
	(ListLink
		(Concept "irritated") ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "confused")
	(ListLink
		(Concept "confused") ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "recoil")
	(ListLink
		(Concept "recoil")   ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

(DefineLink
	(DefinedSchema "surprised")
	(ListLink
		(Concept "surprised") ; blender animation name.
		(Number 4)           ; duration, seconds
		(Number 0.3)         ; intensity
	))

; -----
; Grounding of facial expressions by animations in the Eva blender model:
; happy, comprehending, engaged, bored, irritated
; sad, confused, recoil, surprised

; Syntactic category of facial expression imperative
; "express" is used with "Smile!", "Frown!", etc.
(ReferenceLink (WordNode "express") (GroundedPredicate "py:do_emotion"))
(ReferenceLink (WordNode "show") (GroundedPredicate "py:do_emotion"))
; "look" is used with "Look happy!"
(ReferenceLink (WordNode "look") (GroundedPredicate "py:do_emotion"))
(InheritanceLink (GroundedPredicate "py:do_emotion")
	(ConceptNode "pred-express"))

; Syntactic category of facial-expression schema.
(InheritanceLink (DefinedSchema "happy") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "sad") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "comprehending") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "engaged") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "bored") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "irritated") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "confused") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "recoil") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "surprised") (ConceptNode "schema-express"))

; Syntactic structure of facial-expression imperatives.
(EvaluationLink
	(PredicateNode "express-action")
	(ListLink
		(ConceptNode "pred-express")
		(ConceptNode "schema-express")))

; Currently supported facial animations on the Eva blender model.
; These must be *exactly* as named; these are sent directly to the
; ROS beldner API: ; happy, comprehending, engaged, bored, irritated
; sad, confused, recoil, surprised

; Groundings
(ReferenceLink (WordNode "smile")  (DefinedSchema "happy"))
(ReferenceLink (WordNode "frown")  (DefinedSchema "sad"))
(ReferenceLink (WordNode "recoil") (DefinedSchema "recoil"))

; Look happy! -- adjectives
(ReferenceLink (WordNode "happy")        (DefinedSchema "happy"))
(ReferenceLink (WordNode "sad")          (DefinedSchema "sad"))
(ReferenceLink (WordNode "comprehending")(DefinedSchema "comprehending"))
(ReferenceLink (WordNode "engaged")      (DefinedSchema "engaged"))
(ReferenceLink (WordNode "bored")        (DefinedSchema "bored"))
(ReferenceLink (WordNode "irritated")    (DefinedSchema "irritated"))
(ReferenceLink (WordNode "confused")     (DefinedSchema "confused"))
(ReferenceLink (WordNode "surprised")    (DefinedSchema "surprised"))

; Show happiness! -- nouns
(ReferenceLink (WordNode "happiness")    (DefinedSchema "happy"))
(ReferenceLink (WordNode "sadness")      (DefinedSchema "sad"))
(ReferenceLink (WordNode "comprehension")(DefinedSchema "comprehending"))
(ReferenceLink (WordNode "engagement")   (DefinedSchema "engaged"))
(ReferenceLink (WordNode "boredom")      (DefinedSchema "bored"))
(ReferenceLink (WordNode "irritation")   (DefinedSchema "irritated"))
(ReferenceLink (WordNode "confusion")    (DefinedSchema "confused"))
(ReferenceLink (WordNode "surprise")     (DefinedSchema "surprised"))

;--------------------------------------------------------------------
