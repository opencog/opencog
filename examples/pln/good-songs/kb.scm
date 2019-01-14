;; Authors

(define author-1 (Concept "Metal Head"))
(define author-2 (Concept "Electronic Kundalini"))
(define author-3 (Concept "The Underground Aliens"))

;; Songs

(define song-1 (Concept "Dust or bust"))
(define song-2 (Concept "From harmony to noise and back"))
(define song-3 (Concept "The multiverse within"))
(define song-4 (Concept "Dextrose is my bitch"))
(define song-5 (Concept "I cannot wait to see you with my six eyes"))
(define song-6 (Concept "Symphony of a multidimensional time"))

;; Listeners

(define john (Concept "John"))
(define marry (Concept "Marry"))

;; Authoring

(define composed-by (Predicate "composed-by"))

(define song-1-composed-by-author-1
  (Evaluation (stv 1 1) composed-by (List song-1 author-1)))
(define song-2-composed-by-author-1
  (Evaluation (stv 1 1) composed-by (List song-2 author-1)))
(define song-3-composed-by-author-2
  (Evaluation (stv 1 1) composed-by (List song-3 author-2)))
(define song-4-composed-by-author-2
  (Evaluation (stv 1 1) composed-by (List song-4 author-2)))
(define song-5-composed-by-author-3
  (Evaluation (stv 1 1) composed-by (List song-5 author-3)))
(define song-6-composed-by-author-3
  (Evaluation (stv 1 1) composed-by (List song-6 author-3)))

;; Listener's preferences

(define like (Predicate "like"))

(define john-like-song-2
  (Evaluation (stv 1 1) like (List john song-2)))
(define marry-like-song-3
  (Evaluation (stv 1 1) like (List marry song-3)))
(define marry-like-song-6
  (Evaluation (stv 1 1) like (List marry song-6)))

;; If listener L likes song S from author A, then L is likely to like
;; other songs from A

(define L (Variable "$listener"))
(define S (Variable "$song"))
(define OS (Variable "$other-song"))
(define A (Variable "$author"))
(define ct (Type "ConceptNode"))

(define listener-like-song-from-same-author
  (ImplicationScope (stv 0.9 1)
    (VariableList
      (TypedVariable L ct)
      (TypedVariable S ct)
      (TypedVariable A ct))
    (And
      (Evaluation like (List L S))
      (Evaluation composed-by (List S A)))
    (ImplicationScope
      (TypedVariable OS ct)
      (Evaluation composed-by (List OS A))
      (Evaluation like (List L OS)))))
