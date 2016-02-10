;
; eva.scm
;
; Hacky scaffolding for talking with Hanson Robotics Eva.

;--------------------------------------------------------------------
(use-modules (opencog) (opencog nlp) (opencog query) (opencog exec))
(use-modules (opencog nlp fuzzy))
(load "../relex2logic/rule-utils.scm")

; Must load the rulebase before running eva; see bug
; https://github.com/opencog/opencog/issues/2021 for details
; XXX fixme -- we should not need to load either relex2logic or
; the rules right here, since the code in this module does not depend
; directly on thes.
(use-modules (opencog nlp relex2logic))
(load-r2l-rulebase)

(load "imperative.scm")

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
