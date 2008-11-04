scm
;
; type-definitions.scm
;
; A set of type definitions that hold for some of the more "obscure"
; RelEx types. These are "anti-syntactic sugar", and avoid the need
; for new atom types, such as "GenderNode" or "TenseNode", etc.
;
; Copyright (c) 2008 Linas Vepstas
;
; XXX This list is *probably* incomplete, and needs to be reviewed! XXX
; In particular, the tense list is incomplete. The par-of-speech list 
; might be incomplete.
; The list of inflections is incomplete.
; The list of entities (person, locatin, date, money) is incomplete.
; The place to check for completeness is on the RelEx wiki page,
; documenting these things.
; 
; In alphabetic order.
;
;; -------------------------------------------------------------------
; DEFINITE-FLAG in RelEx
(InheritenceLink
	(DefinedLinguisticConceptNode "definite")
	(ConceptNode "DefinedLinguisticConcept-Determiner")
)

;; -------------------------------------------------------------------
; gender in RelEx
;
(InheritenceLink
	(DefinedLinguisticConceptNode "feminine")
	(ConceptNode "DefinedLinguisticConcept-Gender")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "masculine")
	(ConceptNode "DefinedLinguisticConcept-Gender")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "neuter")
	(ConceptNode "DefinedLinguisticConcept-Gender")
)

;; -------------------------------------------------------------------
; HYP in RelEx
;
(InheritenceLink
	(DefinedLinguisticConceptNode "hyp")
	(ConceptNode "DefinedLinguisticConcept-Hypothetical")
)

;; -------------------------------------------------------------------
; inflection-TAG in RelEx
;
; XXX these should probably be replaced by a new InflectionNode,
; as there will likely be a lot of these, too many to list ? XXX
(InheritenceLink
	(DefinedLinguisticConceptNode ".a")
	(ConceptNode "DefinedLinguisticConcept-Inflection")
)

(InheritenceLink
	(DefinedLinguisticConceptNode ".f")
	(ConceptNode "DefinedLinguisticConcept-Inflection")
)

(InheritenceLink
	(DefinedLinguisticConceptNode ".g")
	(ConceptNode "DefinedLinguisticConcept-Inflection")
)

(InheritenceLink
	(DefinedLinguisticConceptNode ".n")
	(ConceptNode "DefinedLinguisticConcept-Inflection")
)

(InheritenceLink
	(DefinedLinguisticConceptNode ".v")
	(ConceptNode "DefinedLinguisticConcept-Inflection")
)

;; -------------------------------------------------------------------
; noun_number in RelEx
(InheritenceLink
	(DefinedLinguisticConceptNode "plural")
	(ConceptNode "DefinedLinguisticConcept-NounNumber")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "singular")
	(ConceptNode "DefinedLinguisticConcept-NounNumber")
)

;; -------------------------------------------------------------------
; Part-of-speech
(InheritenceLink
	(DefinedLinguisticConceptNode "det")
	(ConceptNode "DefinedLinguisticConcept-PartOfSpeech")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "noun")
	(ConceptNode "DefinedLinguisticConcept-PartOfSpeech")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "prep")
	(ConceptNode "DefinedLinguisticConcept-PartOfSpeech")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "punctuation")
	(ConceptNode "DefinedLinguisticConcept-PartOfSpeech")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "verb")
	(ConceptNode "DefinedLinguisticConcept-PartOfSpeech")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "WORD")
	(ConceptNode "DefinedLinguisticConcept-PartOfSpeech")
)

;; -------------------------------------------------------------------
; person-FLAG in RelEx
(InheritenceLink
	(DefinedLinguisticConceptNode "person")
	(ConceptNode "DefinedLinguisticConcept-Person")
)

;; -------------------------------------------------------------------
; PRONOUN-FLAG in RelEx
(InheritenceLink
	(DefinedLinguisticConceptNode "pronoun")
	(ConceptNode "DefinedLinguisticConcept-Pronoun")
)

;; -------------------------------------------------------------------
; tense in RelEx
; XXX this list is highly incomplete
;
(InheritenceLink
	(DefinedLinguisticConceptNode "past")
	(ConceptNode "DefinedLinguisticConcept-Tense")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "present")
	(ConceptNode "DefinedLinguisticConcept-Tense")
)

(InheritenceLink
	(DefinedLinguisticConceptNode "progressive")
	(ConceptNode "DefinedLinguisticConcept-Tense")
)

.
exit
