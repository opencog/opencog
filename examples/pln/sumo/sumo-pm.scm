;; Execute step by step inference to prove that an SUMO Organization
;; has at least one member.
;;
;; The reasoning goes as follows
;;
;; 1. An Organization is a Group, Org1-1 is an instance of an
;; Organization, therefore it is an instance of a Group.
;;
;; 2. A Group is a Collection, from 1. Org1-1 is an instance of a
;; Group, therefore it is an instance of a Collection.
;;
;; 3. A Collection has at least one member, since Org1-1 is a
;; Collection it has at least one member.
;;
;; Warning: if each step takes too long start guile loading only
;; Merge.scm instead all-sumo-labeled-kb.scm (don't forget to copy
;; Merge.scm from`<EXTERNAL_TOOLS>/SUMO_importer/sumo/output` to the
;; current directory).
;;
;; guile --no-auto-compile -l Merge.scm

(use-modules (opencog logger))
(cog-logger-set-level! "debug")

(load "pln-config.scm")

;; Org1-1 is an Organization
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Org1-1" (stv 0.010000 1.000000))
  (ConceptNode "Organization" (stv 0.010000 1.000000))
)

;; 1. Apply conditional instantiation using the MemberLink above and
;; inheritance KB-15619, to obtain
;;
;; (MemberLink (stv 1 0.99)
;;   (ConceptNode "Org1-1")
;;   (ConceptNode "Group"))
(meta-bind conditional-full-instantiation-inheritance-meta-rule)

;; 2. Apply conditional instantiation using the MemberLink in 1. and
;; inheritance KB-16125, to obtain
;;
;; (MemberLink (stv 1 0.99)
;;   (ConceptNode "Org1-1")
;;   (ConceptNode "Collection"))
(meta-bind conditional-full-instantiation-inheritance-meta-rule)

;; 3. Apply conditional instantiation using 2. and KB-16709 to produce
;; the target
;;
;; (ExistsLink
;;   (TypedVariableLink
;;     (VariableNode "?MEMBER")
;;     (TypeChoice
;;       (TypeNode "ConceptNode")
;;       (TypeNode "SchemaNode")
;;       (TypeNode "PredicateNode")
;;     )
;;   )
;;   (MemberLink
;;     (VariableNode "?MEMBER")
;;     (ConceptNode "Org1-1" (stv 0.010000 1.000000))
;;   )
;; )
(meta-bind conditional-full-instantiation-implication-scope-meta-rule)
