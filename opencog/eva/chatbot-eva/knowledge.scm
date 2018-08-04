;
; knowledge.scm
;
; Knowledge representation of grounded word meanings.
; That is, a "domain ontology" for the robot domain.
;
; This file encodes associations between words and ground truth.
; Certain nouns are linked to certain physical objects; certain verbs
; are linked to specific robot actions, while adjectives and adverbs
; alter the interpretation of the nouns and verbs.
;
; The `semantics.scm` file (and other files) contain rules that try
; to associate parsed natural-langauge sentences with the knowledge
; that is represented here.
;
; This file contains not just one grounding, but two:  one grounding
; is used to map words to physical robot movements.  Another grounding
; is used to map words to a self-representation of the robot state.
;
; The first grounding results in movement commands that are transmitted
; to the robot via python calls; these cause ROS messages to be sent to
; the blender model, which in turn drives robot motors.
;
; The second grounding causes a self-representation of the robot to
; change. The self-representation or self-model resides in the
; atomspace, where it can be queried, examined by other opencog agents
; and processes. In particular, the self model allows the chatbot to
; answer questions about what it is doing: it makes the robot
; "self-aware".
;
; The general knowledge representation used here has a strong linguistic
; feel to it.  Its lexical at the single-word level, and its syntactic
; at the multi-word level.  The general structure is as follows:
;
; Single words are associated with atomspace objects, typically numeric
; values or the names of subroutines. For example, the direction adverbs
; "right", "left" are associated with 3D numeric XYZ values, while verbs
; such as "look", "turn" are associated with python function names.
; Emotion adjectives such as "happy" are associated with blender
; animation names and parameters.
;
; Not all subroutines can accept all possible arguments: the routine
; used to turn the head (rotate the neck) of the robot only accepts
; XYZ directions; it cannot take the names of animations as an argument.
; Likewise, the facial animation driver can only take animation names;
; it cannot take XYZ directions.  These constraints are implemented
; by specifying a "syntax": actions (verbs) can link only to certain
; parameters (adverbs, adjectives).  The syntax is link-grammar-like:
; there is a link-name that links together a routine and its allowed
; range of arguments.
;
; This "syntactic" structure then suggests that actions and paramters be
; grouped into ontological (taxonomic) categories: all directions are
; grouped together into a "direction" category; all facial animations
; are grouped into an "emotion" category.  All verbs that can take a
; direction are grouped into a direction-taking-verb category, and so
; on.
;
; Word-to-object associations are done with a ReferenceLink
; Taxonomic is-a relations are done with an InheritanceLink
; Syntactic structure is encoded with an EvaluationLink.
;--------------------------------------------------------------------
;
; (DefinedPredicate "Show class expression") is defined in eva-behavior,
; so we have to import that...
(use-modules (opencog eva-behavior))
;--------------------------------------------------------------------
; Global knowledge about spatial directions.  The coordinate system
; is specific to the HR robot head.  Distance in meters, the origin
; of the system is behind the eyes, middle of head.  "forward" is the
; object the chest is facing.
;
; XXX This is incorrect, just right now; its too simple, and interacts
; badly with the face tracker.  The directions need to be general
; 2D areas.  For imperative commands, we need to pick some random point
; out of the 2D area. To answer questions, we need to compare the 2D
; extent to the current gaze direction.  The gaze direction can be
; gotten from the space-time server, (octree server) -- someone needs
; to hook this up.  XXX FIXME.
;
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
		(Number 0)    ; y is right
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
(ReferenceLink (WordNode "look") (DefinedPredicate "Gaze command"))
(ReferenceLink (WordNode "turn") (DefinedPredicate "Look command"))

; Syntactic category of imperative verbs.
(InheritanceLink (DefinedPredicate "Gaze command")
	(ConceptNode "pred-direction"))
(InheritanceLink (DefinedPredicate "Look command")
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
; ConceptNode, instead.
;
; XXX Remove this -- It should go into the self-model file.

; Knowledge about spatial directions. Pair up words and physical
; directions.
(ReferenceLink (WordNode "up")       (Concept "upward"))
(ReferenceLink (WordNode "down")     (Concept "downward"))
(ReferenceLink (WordNode "right")    (Concept "rightwards"))
(ReferenceLink (WordNode "left")     (Concept "leftwards"))
(ReferenceLink (WordNode "forward")  (Concept "forward"))
(ReferenceLink (WordNode "ahead")    (Concept "forward"))

; Syntactic category of schema. Used for contextual understanding.
(Inheritance (Concept "upward")     (Concept "concept-direction"))
(Inheritance (Concept "downward")   (Concept "concept-direction"))
(Inheritance (Concept "rightwards") (Concept "concept-direction"))
(Inheritance (Concept "leftwards")  (Concept "concept-direction"))
(Inheritance (Concept "forward")    (Concept "concept-direction"))

; Model (self-awareness) knowledge about imperative verbs.
(ReferenceLink (WordNode "look") (AnchorNode "*-gaze-direction-*"))
(ReferenceLink (WordNode "face") (AnchorNode "*-head-direction-*"))
(ReferenceLink (WordNode "turn") (AnchorNode "*-head-direction-*"))

; Model (self-awareness) syntactic category of head-pose verbs
(Inheritance (Anchor "*-gaze-direction-*") (Concept "model-direction"))
(Inheritance (Anchor "*-head-direction-*") (Concept "model-direction"))

; Specifies the syntactic structure for self-model (self-awareness).
(EvaluationLink
	(PredicateNode "turn-model")
	(ListLink
		(ConceptNode "model-direction")
		(ConceptNode "concept-direction")))

;--------------------------------------------------------------------
;--------------------------------------------------------------------
; Similar to above, but for sentences such as "look at me" or "turn
; towards me". As before, this is able to convert from these English
; sentences, into a structured command language. It makes use of the
; (DefinedSchema "current-speaker") in self-model.scm, and also of
; (DefinedPredicate "Look-at-thing cmd") to perform that action.

; Global knowledge about word-meaning.
; Knowledge about things in the environemnt.
; "me" refers to the person who last spoke (and said "look at me")
; "him"/"her" - any other visible person in the room
; this/that - best-guess salient point
(ReferenceLink (WordNode "me")       (DefinedSchema "current-speaker"))

(ReferenceLink (WordNode "her")      (DefinedSchema "other-speaker"))
(ReferenceLink (WordNode "him")      (DefinedSchema "other-speaker"))

(ReferenceLink (WordNode "this")     (DefinedSchema "current-salient"))
(ReferenceLink (WordNode "that")     (DefinedSchema "current-salient"))
(ReferenceLink (WordNode "here")     (DefinedSchema "current-salient"))
(ReferenceLink (WordNode "there")    (DefinedSchema "current-salient"))

; Syntactic category of schema. Used for contextual understanding.
(Inheritance (DefinedSchema "current-speaker")  (Concept "schema-thing"))
(Inheritance (DefinedSchema "other-speaker")    (Concept "schema-thing"))
(Inheritance (DefinedSchema "current-salient")  (Concept "schema-thing"))

; Physical (motor control) knowledge about imperative look-at-thing verbs.
(ReferenceLink (WordNode "look") (DefinedPredicate "Look-at-thing cmd"))
(ReferenceLink (WordNode "turn") (DefinedPredicate "Look-at-thing cmd"))
(ReferenceLink (WordNode "face") (DefinedPredicate "Look-at-thing cmd"))

; Syntactic category of imperative look-at-thing verbs.
(Inheritance (DefinedPredicate "Look-at-thing cmd") (Concept "look-at-cmd"))

; Syntactic structure of "look at thing" action-knowledge --
; Specifies the syntactic structure for physical motor-control commands.
(EvaluationLink
	(PredicateNode "look-at-action")
	(ListLink
		(ConceptNode "look-at-cmd")
		(ConceptNode "schema-thing")))

; XXX FIXME -- Implement-me, actually -- need to do the above, but for
; the self-model, rather than the direct robot action.  The self-model
; is not being updated by these imperatives.
;--------------------------------------------------------------------
;--------------------------------------------------------------------
; Emotional expression semantics (groundings) for robot control
;
; The ListLink provides the arguments to the
; (DefinedPredicate "Show class expression")
; The `expression class` lines up with the config parameter `imperative`
; in the cfg-eva.scm file, which is used to control the strength and
; duration of the expression (randomly chosen)
(DefineLink
	(DefinedSchema "afraid")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "afraid")     ; blender animation name.
	))

(DefineLink
	(DefinedSchema "amused")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "amused")     ; blender animation name.
	))

(DefineLink
	(DefinedSchema "bored")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "bored")      ; blender animation name.
	))

(DefineLink
	(DefinedSchema "comprehending")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "comprehending") ; blender animation name.
	))

(DefineLink
	(DefinedSchema "confused")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "confused")   ; blender animation name.
	))

(DefineLink
	(DefinedSchema "engaged")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "engaged")    ; blender animation name.
	))

(DefineLink
	(DefinedSchema "happy")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "happy")      ; blender animation name.
	))

(DefineLink
	(DefinedSchema "irritated")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "irritated")  ; blender animation name.
	))

(DefineLink
	(DefinedSchema "recoil")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "recoil")     ; blender animation name.
	))

(DefineLink
	(DefinedSchema "sad")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "sad")        ; blender animation name.
	))

(DefineLink
	(DefinedSchema "surprised")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "surprised")  ; blender animation name.
	))

(DefineLink
	(DefinedSchema "worry")
	(ListLink
		(Concept "imperative") ; expression class
		(Concept "worry")      ; blender animation name.
	))

; -----
; Grounding of words for facial expressions by animations in the Eva
; blender model: happy, sad, confused, etc. See below for full list.

; XXX a bunch of verb synonyms -- handled manually. These should be
; automated via synonymous phrase support. Total hack, needs fixing.
; Each verb here must also be handled by a corresponding rule in the
; `imperative-rules.scm` file.
;
; `express-action` is used with "Smile!", "Frown!", etc.
; It lines up with `single-word-express-rule` and must be a WordNode
; so that `obj-semantics-template` works correctly.
(ReferenceLink (Word "express-action") (DefinedPredicate "Show class expression"))

(ReferenceLink (WordNode "dramatize") (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "emote")   (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "enact")   (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "express") (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "feign")   (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "impersonate")(DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "mime")    (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "mimic")   (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "portray") (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "pretend") (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "show")    (DefinedPredicate "Show class expression"))
; "look" is used with "Look happy!"
(ReferenceLink (WordNode "act")     (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "be")      (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "look")    (DefinedPredicate "Show class expression"))
(ReferenceLink (WordNode "play")    (DefinedPredicate "Show class expression"))

; Currently supported facial animations on the Eva blender model.
; These must be *exactly* as named; these are sent directly to the
; ROS blender API:
; rostopic echo /blender_api/available_emotion_states
; ['irritated', 'happy', 'recoil', 'surprised', 'sad', 'confused',
;  'worry', 'bored', 'engaged', 'amused', 'comprehending', 'afraid']


; Groundings
(ReferenceLink (WordNode "smile")  (DefinedSchema "happy"))
(ReferenceLink (WordNode "frown")  (DefinedSchema "sad"))
(ReferenceLink (WordNode "recoil") (DefinedSchema "recoil"))
(ReferenceLink (WordNode "worry")  (DefinedSchema "worry"))

; XXX FIXME ... the list below is duplicated twice, once as adjectives
; and once as nouns.  This is partly because relex normalization is
; not being correctly used, and/or R2L in its current form is not
; quite usable for this (it's too fragile, among other things).
;
; XXX FIXME -- this list contains lots of synonyms; needs to be replaced
; by proper synonym support.
;
; XXX Note that some synonyms have multiple "meanings" e.g. "anguish"
; could map to either "worry" or to "sad" expressions.  So, need to deal
; with that. Also: emotions are graded: some call for much milder
; expressions than others -- right now, the expressions pick a random
; strength out of a range. e.g. "meloncholy" should ba a mild "sad".
;
; Some, like "disorientation", need to couple confusion with eye and
; head movements. (i.e. the "search empty room" behavior tree)
;
; "amused is commented out, because we want to use the amused gesture,
; instead of the amuased expression. The gesture is stronger, quirkier,
; and more interesting to look at.
;
; Look happy! -- adjectives
(ReferenceLink (WordNode "afraid")       (DefinedSchema "afraid"))
(ReferenceLink (WordNode "amazed")       (DefinedSchema "surprised"))
; (ReferenceLink (WordNode "amused")       (DefinedSchema "amused"))
(ReferenceLink (WordNode "angered")      (DefinedSchema "irritated"))
(ReferenceLink (WordNode "angry")        (DefinedSchema "irritated"))
(ReferenceLink (WordNode "anguished")    (DefinedSchema "worry"))
(ReferenceLink (WordNode "annoyed")      (DefinedSchema "irritated"))
(ReferenceLink (WordNode "anxious")      (DefinedSchema "worry"))
(ReferenceLink (WordNode "baffled")      (DefinedSchema "confused"))
(ReferenceLink (WordNode "befuddled")    (DefinedSchema "confused"))
(ReferenceLink (WordNode "bitter")       (DefinedSchema "sad"))
(ReferenceLink (WordNode "bored")        (DefinedSchema "bored"))
(ReferenceLink (WordNode "bothered")     (DefinedSchema "irritated"))
(ReferenceLink (WordNode "chagrined")    (DefinedSchema "confused"))
; (ReferenceLink (WordNode "charmed")      (DefinedSchema "amused"))
(ReferenceLink (WordNode "cheered")      (DefinedSchema "amused"))
(ReferenceLink (WordNode "cheerful")     (DefinedSchema "happy"))
(ReferenceLink (WordNode "comprehending")(DefinedSchema "comprehending"))
(ReferenceLink (WordNode "comprehensive")(DefinedSchema "comprehending"))
(ReferenceLink (WordNode "concerned")    (DefinedSchema "worry"))
(ReferenceLink (WordNode "confused")     (DefinedSchema "confused"))
(ReferenceLink (WordNode "dazed")        (DefinedSchema "confused"))
(ReferenceLink (WordNode "delighted")    (DefinedSchema "happy"))
(ReferenceLink (WordNode "demoralized")  (DefinedSchema "confused"))
(ReferenceLink (WordNode "discerning")   (DefinedSchema "comprehending"))
(ReferenceLink (WordNode "disgusted")    (DefinedSchema "recoil"))
(ReferenceLink (WordNode "disinterested")(DefinedSchema "bored"))
(ReferenceLink (WordNode "distracted")   (DefinedSchema "confused"))
(ReferenceLink (WordNode "distressed")   (DefinedSchema "worry"))
(ReferenceLink (WordNode "ecstatic")     (DefinedSchema "happy"))
(ReferenceLink (WordNode "elated")       (DefinedSchema "happy"))
(ReferenceLink (WordNode "embarassed")   (DefinedSchema "confused"))
(ReferenceLink (WordNode "engaged")      (DefinedSchema "engaged"))
(ReferenceLink (WordNode "enthusiastic") (DefinedSchema "engaged"))
(ReferenceLink (WordNode "entranced")    (DefinedSchema "engaged"))
(ReferenceLink (WordNode "fatigued")     (DefinedSchema "bored"))
(ReferenceLink (WordNode "fearful")      (DefinedSchema "afraid"))
(ReferenceLink (WordNode "frightened")   (DefinedSchema "afraid"))
(ReferenceLink (WordNode "glad")         (DefinedSchema "happy"))
(ReferenceLink (WordNode "happy")        (DefinedSchema "happy"))
(ReferenceLink (WordNode "heartbroken")  (DefinedSchema "sad"))
(ReferenceLink (WordNode "interested")   (DefinedSchema "engaged"))
(ReferenceLink (WordNode "irked")        (DefinedSchema "irritated"))
(ReferenceLink (WordNode "irritated")    (DefinedSchema "irritated"))
(ReferenceLink (WordNode "joyful")       (DefinedSchema "happy"))
(ReferenceLink (WordNode "joyous")       (DefinedSchema "happy"))
(ReferenceLink (WordNode "knowing")      (DefinedSchema "comprehending"))
(ReferenceLink (WordNode "knowlegable")  (DefinedSchema "comprehending"))
(ReferenceLink (WordNode "meloncholy")   (DefinedSchema "sad"))
(ReferenceLink (WordNode "meloncholic")  (DefinedSchema "sad"))
(ReferenceLink (WordNode "mournful")     (DefinedSchema "sad"))
(ReferenceLink (WordNode "muddled")      (DefinedSchema "confused"))
(ReferenceLink (WordNode "nauseated")    (DefinedSchema "recoil"))
(ReferenceLink (WordNode "nauseous")     (DefinedSchema "recoil"))
(ReferenceLink (WordNode "overjoyed")    (DefinedSchema "happy"))
(ReferenceLink (WordNode "perplexed")    (DefinedSchema "confused"))
(ReferenceLink (WordNode "perturbed")    (DefinedSchema "confused"))
(ReferenceLink (WordNode "petrified")    (DefinedSchema "afraid"))
(ReferenceLink (WordNode "pleased")      (DefinedSchema "amused"))
(ReferenceLink (WordNode "provoked")     (DefinedSchema "irritated"))
(ReferenceLink (WordNode "puzzled")      (DefinedSchema "confused"))
(ReferenceLink (WordNode "quesy")        (DefinedSchema "recoil"))
(ReferenceLink (WordNode "relaxed")      (DefinedSchema "amused"))
(ReferenceLink (WordNode "repelled")     (DefinedSchema "recoil"))
(ReferenceLink (WordNode "revulsed")     (DefinedSchema "recoil"))
(ReferenceLink (WordNode "revolted")     (DefinedSchema "recoil"))
(ReferenceLink (WordNode "sad")          (DefinedSchema "sad"))
(ReferenceLink (WordNode "scared")       (DefinedSchema "afraid"))
(ReferenceLink (WordNode "sorrowful")    (DefinedSchema "sad"))
(ReferenceLink (WordNode "sorry")        (DefinedSchema "sad"))
(ReferenceLink (WordNode "startled")     (DefinedSchema "surprised"))
(ReferenceLink (WordNode "surprised")    (DefinedSchema "surprised"))
(ReferenceLink (WordNode "terrorized")   (DefinedSchema "afraid"))
(ReferenceLink (WordNode "thrilled")     (DefinedSchema "happy"))
(ReferenceLink (WordNode "tired")        (DefinedSchema "bored"))
(ReferenceLink (WordNode "troubled")     (DefinedSchema "irritated"))
(ReferenceLink (WordNode "unsettled")    (DefinedSchema "irritated"))
(ReferenceLink (WordNode "upset")        (DefinedSchema "irritated"))
(ReferenceLink (WordNode "wistful")      (DefinedSchema "sad"))
(ReferenceLink (WordNode "worried")      (DefinedSchema "worry"))

; Show happiness! -- nouns
; Feign amusement!
(ReferenceLink (WordNode "affection")    (DefinedSchema "engaged"))
(ReferenceLink (WordNode "amazement")    (DefinedSchema "surprised"))
; (ReferenceLink (WordNode "amusement")    (DefinedSchema "amused"))
(ReferenceLink (WordNode "anguish")      (DefinedSchema "worry"))
(ReferenceLink (WordNode "antipathy")    (DefinedSchema "recoil"))
(ReferenceLink (WordNode "anxiousness")  (DefinedSchema "worry"))
(ReferenceLink (WordNode "apprehension") (DefinedSchema "worry"))
(ReferenceLink (WordNode "awareness")    (DefinedSchema "comprehending"))
(ReferenceLink (WordNode "bewilderment") (DefinedSchema "confused"))
(ReferenceLink (WordNode "boredom")      (DefinedSchema "bored"))
(ReferenceLink (WordNode "chagrin")      (DefinedSchema "confused"))
(ReferenceLink (WordNode "confusion")    (DefinedSchema "confused"))
(ReferenceLink (WordNode "comprehension")(DefinedSchema "comprehending"))
(ReferenceLink (WordNode "concern")      (DefinedSchema "worry"))
(ReferenceLink (WordNode "delight")      (DefinedSchema "happy"))
(ReferenceLink (WordNode "disgust")      (DefinedSchema "recoil"))
(ReferenceLink (WordNode "dislike")      (DefinedSchema "recoil"))
(ReferenceLink (WordNode "disorientation")(DefinedSchema "confused"))
(ReferenceLink (WordNode "distaste")     (DefinedSchema "recoil"))
(ReferenceLink (WordNode "distress")     (DefinedSchema "worry"))
(ReferenceLink (WordNode "doubt")        (DefinedSchema "worry"))
(ReferenceLink (WordNode "doubtful")     (DefinedSchema "worry"))
(ReferenceLink (WordNode "embarassment") (DefinedSchema "confused"))
(ReferenceLink (WordNode "engagement")   (DefinedSchema "engaged"))
(ReferenceLink (WordNode "enjoyment")    (DefinedSchema "happy"))
(ReferenceLink (WordNode "enthusiasm")   (DefinedSchema "engaged"))
(ReferenceLink (WordNode "exhuberance")  (DefinedSchema "happy"))
(ReferenceLink (WordNode "euphoria")     (DefinedSchema "happy"))
(ReferenceLink (WordNode "fear")         (DefinedSchema "afraid"))
(ReferenceLink (WordNode "fearfulness")  (DefinedSchema "afraid"))
(ReferenceLink (WordNode "fright")       (DefinedSchema "afraid"))
(ReferenceLink (WordNode "glee")         (DefinedSchema "happy"))
(ReferenceLink (WordNode "greif")        (DefinedSchema "sad"))
(ReferenceLink (WordNode "happiness")    (DefinedSchema "happy"))
(ReferenceLink (WordNode "hatred")       (DefinedSchema "recoil"))
(ReferenceLink (WordNode "heartache")    (DefinedSchema "sad"))
(ReferenceLink (WordNode "heartbreak")   (DefinedSchema "sad"))
(ReferenceLink (WordNode "hopelessness") (DefinedSchema "sad"))
(ReferenceLink (WordNode "intelligence") (DefinedSchema "comprehending"))
(ReferenceLink (WordNode "interest")     (DefinedSchema "engaged"))
(ReferenceLink (WordNode "irritation")   (DefinedSchema "irritated"))
(ReferenceLink (WordNode "jubilation")   (DefinedSchema "happy"))
(ReferenceLink (WordNode "loathing")     (DefinedSchema "recoil"))
(ReferenceLink (WordNode "meloncholy")   (DefinedSchema "sad"))
(ReferenceLink (WordNode "mournfulness") (DefinedSchema "sad"))
(ReferenceLink (WordNode "passion")      (DefinedSchema "engaged"))
(ReferenceLink (WordNode "perplexity")   (DefinedSchema "confused"))
(ReferenceLink (WordNode "puzzlement")   (DefinedSchema "confused"))
(ReferenceLink (WordNode "revulsion")    (DefinedSchema "recoil"))
(ReferenceLink (WordNode "sadness")      (DefinedSchema "sad"))
(ReferenceLink (WordNode "sorrow")       (DefinedSchema "sad"))
(ReferenceLink (WordNode "surprise")     (DefinedSchema "surprised"))
(ReferenceLink (WordNode "uneasiness")   (DefinedSchema "worry"))
(ReferenceLink (WordNode "worriment")    (DefinedSchema "worry"))
(ReferenceLink (WordNode "worry")        (DefinedSchema "worry"))

; -----
; Syntactic category of robot-control facial expression imperative
(InheritanceLink (DefinedPredicate "Show class expression")
	(ConceptNode "pred-express"))

; Syntactic category of robot-control facial-expression schema.
; There are 12 of these in the current blender model.
(InheritanceLink (DefinedSchema "afraid")    (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "amused")    (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "bored")     (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "comprehending") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "confused")  (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "engaged")   (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "happy")     (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "irritated") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "recoil")    (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "sad")       (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "surprised") (ConceptNode "schema-express"))
(InheritanceLink (DefinedSchema "worry")     (ConceptNode "schema-express"))

; Syntactic structure of robot-control facial-expression imperatives.
(EvaluationLink
	(PredicateNode "express-action")
	(ListLink
		(ConceptNode "pred-express")
		(ConceptNode "schema-express")))

;--------------------------------------------------------------------
; Duplicate of the above, except that this is for use in controlling
; the self-model, rather than the physical motors.
;
; XXX Remove this -- this si supposed to ba a part of the action
; orchestrateor, i.e. the self model is about what the orchestrator
; actually did, not what it was told to do.

(Reference (Word "express") (Anchor "*-facial-expression-*"))

; Model (self-awareness) syntactic category of facial expression verbs
(Inheritance (Anchor "*-facial-expression-*") (Concept "model-expression"))

; Syntactic structure of self-model facial-expression imperatives.
(EvaluationLink
	(PredicateNode "express-model")
	(ListLink
		(ConceptNode "model-expression")
		(ConceptNode "schema-express")))

;--------------------------------------------------------------------
; Support for gestures (shake, nod, blink) as oposed to emotial
; expressions.
;
; rostopic echo /blender_api/available_gestures
; data: ['all', 'amused', 'blink', 'blink-micro', 'blink-relaxed',
; 'blink-sleepy', 'nod-1', 'nod-2', 'nod-3', 'shake-2', 'shake-3',
; 'thoughtful', 'yawn-1']
;
;--------------------------------------------------------------------
; Gesture expression semantics (groundings) for robot control
;
; The ListLink provides the arguments to the
; (DefinedPredicate "Show class gesture")
; The `gesture class` lines up with the config parameter `imperative`
; in the cfg-eva.scm file, which is used to control the strength and
; speed of the gesture (randomly chosen)
(DefineLink
	(DefinedSchema "amused-gest")
	(ListLink
		(Concept "imperative") ; gesture class
		(Concept "amused")     ; blender animation name.
	))

(DefineLink
	(DefinedSchema "blink")
	(ListLink
		(Concept "imperative") ; gesture class
		(Concept "blink")      ; blender animation name.
	))

(DefineLink
	(DefinedSchema "nod-1")
	(ListLink
		(Concept "imperative") ; gesture class
		(Concept "nod-1")      ; blender animation name.
	))

(DefineLink
	(DefinedSchema "shake-2")
	(ListLink
		(Concept "imperative") ; gesture class
		(Concept "shake-2")    ; blender animation name.
	))

(DefineLink
	(DefinedSchema "thoughtful")
	(ListLink
		(Concept "imperative") ; gesture class
		(Concept "thoughtful") ; blender animation name.
	))

(DefineLink
	(DefinedSchema "yawn-1")
	(ListLink
		(Concept "imperative") ; gesture class
		(Concept "yawn-1")     ; blender animation name.
	))

; -----
; Grounding of words for facial gestures by animations in the Eva
; blender model: blink, nod, shake, etc. See below for full list.

(ReferenceLink (WordNode "amused")     (DefinedSchema "amused-gest"))
(ReferenceLink (WordNode "amusement")  (DefinedSchema "amused-gest"))
(ReferenceLink (WordNode "bob")        (DefinedSchema "nod-1"))
(ReferenceLink (WordNode "blink")      (DefinedSchema "blink"))
(ReferenceLink (WordNode "charmed")    (DefinedSchema "amused-gest"))
(ReferenceLink (WordNode "nod")        (DefinedSchema "nod-1"))
(ReferenceLink (WordNode "shake")      (DefinedSchema "shake-2"))
(ReferenceLink (WordNode "thinking")   (DefinedSchema "thoughtful"))
(ReferenceLink (WordNode "thought")    (DefinedSchema "thoughtful"))
(ReferenceLink (WordNode "thoughtful") (DefinedSchema "thoughtful"))
(ReferenceLink (WordNode "thoughtfulness") (DefinedSchema "thoughtful"))
(ReferenceLink (WordNode "yawn")       (DefinedSchema "yawn-1"))

; -----
; Syntactic category of robot-control facial gesture imperative
; The (WordNode "gesture-action") is not really a "word"; it must
; be identical to the "word" in `single-word-gesture-rule` and it
; must be a WordNode to make `obj-semantics-template` happy.
;
; Other verbs: "Look thoughtful"
(ReferenceLink (Word "gesture-action") (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "act")            (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "be")             (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "look")           (DefinedPredicate "Show class gesture"))

; These are more-or-less the same verbs as before, but coupled to
; gestures, so that "feign thoughtfulness" and "mime amusement" work
; for the gestures.
(ReferenceLink (Word "dramatize")   (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "emote")       (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "enact")       (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "express")     (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "feign")       (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "impersonate") (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "mime")        (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "mimic")       (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "portray")     (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "pretend")     (DefinedPredicate "Show class gesture"))
(ReferenceLink (Word "show")        (DefinedPredicate "Show class gesture"))

(InheritanceLink (DefinedPredicate "Show class gesture")
	(ConceptNode "pred-gesture"))

;
(InheritanceLink (DefinedSchema "amused-gest")(Concept "schema-gesture"))
(InheritanceLink (DefinedSchema "blink")      (Concept "schema-gesture"))
(InheritanceLink (DefinedSchema "nod-1")      (Concept "schema-gesture"))
(InheritanceLink (DefinedSchema "shake-2")    (Concept "schema-gesture"))
(InheritanceLink (DefinedSchema "thoughtful") (Concept "schema-gesture"))
(InheritanceLink (DefinedSchema "yawn-1")     (Concept "schema-gesture"))

; Syntactic structure of robot-control facial-gesture imperatives.
(EvaluationLink
	(Predicate "gesture-action")
	(ListLink
		(Concept "pred-gesture")
		(Concept "schema-gesture")))

;--------------------------------------------------------------------
; Knowledge for going into different demo modes

(Define
	(DefinedSchema "reasoning-demo")
	(List (Concept "reasoning-demo")))

(Define
	(DefinedSchema "philosophy-demo")
	(List (Concept "philosophy-demo")))

(Define
	(DefinedSchema "saliency-demo")
	(List (Concept "saliency-demo")))

; The keyword for triggering a demo
(Reference (Word "show") (DefinedPredicate "Do show demo"))

; The keywords to indicate which mode to go
(Reference (Word "reasoning") (DefinedSchema "reasoning-demo"))
(Reference (Word "philosophy") (DefinedSchema "philosophy-demo"))
(Reference (Word "saliency") (DefinedSchema "saliency-demo"))

(Inheritance (DefinedPredicate "Do show demo") (Concept "pred-demo"))

(Inheritance (DefinedSchema "reasoning-demo") (Concept "schema-demo"))
(Inheritance (DefinedSchema "philosophy-demo") (Concept "schema-demo"))
(Inheritance (DefinedSchema "saliency-demo") (Concept "schema-demo"))

(Evaluation
	(Predicate "demo-action")
	(List
		(Concept "pred-demo")
		(Concept "schema-demo")))
