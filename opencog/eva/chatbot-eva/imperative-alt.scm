;
; imperative-alt.scm
;
; Sketch of an alternate imperative-processing architecture.
; The stuff here does not work.  However, it is meant to be
; a generalization or softening of the processing done in
; "imperative.scm".  It is meant to be more robust, more
; scalable, more human-like. Its incomplete, due to some
; design difficulties.
;
; Here's the idea:  The knowledge base consists of a collection
; of parsed sentences that Eva "knows" -- these are ground knowledge.
; Each of these parsed sentences are directly associated with specific
; (mechanical, physical) actions.  When Eva hears a new English-language
; sentence, she performs a fuzzy match, to see if it resembles any of
; the known ground-sentences.  The best fuzzy-match wins, and the
; highest-scoring associated action is performed.
;
; The above seems like a good idea; there is one design difficulty,
; though, preventing current progress.   During the fuzzy match, we
; need to fund specific modifiers (adjectives, direct objects,
; prepositions, etc.) and get values for them.  So, for example,
; in a sentence "look to the left", we have to be able to pick out the
; verb "look", and the direction "left", because these are the primary
; components for physical action.  Thus, if Eva hears "look to the sky",
; we want the fuzzy matcher to identify "to the sky" as the "direction".
; Currently, the fuzzy matcher does not have the ability to pick out
; components in this fashion. Its not clear how to build the fuzzy
; matcher that will do this,. because its not clear of what a good
; example looks like.
;
; The fuzzy matcher has other problems. During fuzzy matching, it would
; be nice to say that certan parts of a graph are highly invariant,
; and must be matched precisely, while others can be loose, and roughly
; matched. These are part of the scoring system. Its not clear how to
; associate the scores with the different parts of the pattern. Are they
; truth values?  Do we need the protoatoms[now renamed values] for this?
;
; Fuzzy matching does give us some ability of maybe learn.  If Eva
; hears  "look up at the sky", and she knows "look up" as grounded
; knowledge, then "at the sky" can be isolated as a correlate of "up",
; and used as a possible synonym.
;
; Handling of synonyms is another problem that this proposed
; fuzzy-matching architecture does not seem to handle cleanly.
; part of matching requires considering synonymous words: e.g. "glance"
; as a synonym for "look", but also synonymous phrases: "take a gander
; at your left". In this example, a phrases is synonymous to a single
; word; more generally, different phrases are synonymous to each other.
; We want to do fuzzy matching, but with synonymy as an intermediate
; step.
;
; The above considerations have halted the progress here.  I'm keeping
; the code around for a little while, though, as a reminder of a
; possible direction to explore.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp) (opencog exec))
(use-modules (opencog nlp fuzzy))
(use-modules (opencog nlp relex2logic))

; Must load the rulebase before running eva; see bug
; https://github.com/opencog/opencog/issues/2021 for details
; XXX fixme -- we should not need to load either relex2logic or
; the rules right here, since the code in this module does not depend
; directly on thes.
(use-modules (opencog nlp relex2logic))
(load-r2l-rulebase)

;--------------------------------------------------------------------
; Global semantic knowledge
; See farther down below; we build a ReferenceLink attaching
; specific parsed sentences to specific actions.

(define (get-interp-node sent-node)
"
  Given a sentence, get the likliest interpretation node for it.
  At this time, it simply returns the very first interpretation.
  Yes, this is a quick hack, needs fixing. XXX FIXME.
"
	(define parse (car (cog-chase-link 'ParseLink 'ParseNode sent-node)))
	(car (cog-chase-link 'InterpretationLink 'InterpretationNode parse)))

(define (get-interp-of-r2l r2l-set-list)
"
  Given a ListLink of r2l-sets, pick out the InterpetationNode from
  each, and return those (as a list).

XXX this may be junk/obsolete, the format of r2l-sets seems to have
changed recently.  I'm confused. Current structure seems to be this:

(ReferenceLink (InterpretationNode \"sentence@f2b..\") (SetLink ...))

but this is not what the code below looks for...
"
	; find-interp takes a single SetLink
	(define (find-interp r2l-set)
		; find-inh returns #f if inh-link is not an InheritanceLink
		; It also returns #f if it is an InheritanceLink, but
		; its first member is not an InterpretationNode
		(define (find-inh inh-link)
			(if (eq? (cog-type inh-link) 'InheritanceLink)
				(eq? 'InterpretationNode
					(cog-type (car (cog-outgoing-set inh-link))))
				#f
			)
		)

		; The find returns (should return) a single InheritanceLink
		; and the first member should be the desired InterpretationNode
		(car (cog-outgoing-set
			(find find-inh (cog-outgoing-set r2l-set))))
	)

	(map find-interp (cog-outgoing-set r2l-set-list))
)

;--------------------------------------------------------------------
;
; These are English-language sentences that I (Eva) understand.
; XXX These are not being used right now; these are meant to be
; fuzzy-matched, in a newer/different design,.... which is
; maybe abandoned right now???
(define known-directives
	(list
		(get-interp-node (car (nlp-parse "look left")))
		(get-interp-node (car (nlp-parse "look right")))
		(get-interp-node (car (nlp-parse "look up")))
		(get-interp-node (car (nlp-parse "look down")))
	))

;--------------------------------------------------------------------

(define (imperative-process-v2 imp)
"
  Process imperative IMP, which should be a SentenceNode.
"

	; Get the r2l-set of the sentence
	(define r2l-set (get-r2l-set-of-sent imp))

	; Get the sentences that are similar to it.
	(define fzset (cog-fuzzy-match r2l-set))

	; Get the InterpretationNode's out of that set.
	(define interp (car (get-interp-of-r2l fzset)))

	; See if it is an interpretation that we know
	(define known (find (lambda (inp) (eq? interp inp)) known-directives))

	(if (eq? #f known)
		(display "I don't know how to do that.\n")
	)

	known
)

;--------------------------------------------------------------------
