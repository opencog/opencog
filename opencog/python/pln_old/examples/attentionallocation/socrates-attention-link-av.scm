(ConceptNode "Socrates@f250f429-5078-4453-8662-c63cc8f58a22" (stv .001 .99)) ; [217]

(ConceptNode "Socrates" (stv .001 .99)) ; [218]

(ConceptNode "man@80d4e852-1b93-4c49-84e1-5a7352b0dcb1" (stv .001 .99)) ; [220]

(ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13" (stv .001 .99)) ; [290]

(ConceptNode "man" (stv .001 .99)) ; [221]

(ConceptNode "air@e3f175ea-c3d1-45cf-883d-a6d2f6a879ac" (stv .001 .99)) ; [292]

(ConceptNode "air" (stv .001 .99)) ; [293]

(ConceptNode "present" (stv .001 .99)) ; [227]

(ListLink (stv .99 .99)
  (ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13") ; [290]
  (ConceptNode "air@e3f175ea-c3d1-45cf-883d-a6d2f6a879ac") ; [292]
) ; [295]

(EvaluationLink (stv .99 .99) (av 1000 0 0)
  (PredicateNode "breathe@218b15b2-52a8-430c-93f6-b4bba225418c") ; [287]
  (ListLink (stv .99 .99)
    (ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13") ; [290]
    (ConceptNode "air@e3f175ea-c3d1-45cf-883d-a6d2f6a879ac") ; [292]
  ) ; [295]
) ; [296]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (PredicateNode "breathe@218b15b2-52a8-430c-93f6-b4bba225418c") ; [287]
  (PredicateNode "breathe") ; [288]
) ; [289]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (ConceptNode "men@83d83a2e-940c-46aa-91f1-a7d2c106ef13") ; [290]
  (ConceptNode "man") ; [221]
) ; [291]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (ConceptNode "air@e3f175ea-c3d1-45cf-883d-a6d2f6a879ac") ; [292]
  (ConceptNode "air") ; [293]
) ; [294]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (PredicateNode "breathe@218b15b2-52a8-430c-93f6-b4bba225418c") ; [287]
  (ConceptNode "present") ; [227]
) ; [297]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (ConceptNode "Socrates@f250f429-5078-4453-8662-c63cc8f58a22") ; [217]
  (ConceptNode "Socrates") ; [218]
) ; [219]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (ConceptNode "man@80d4e852-1b93-4c49-84e1-5a7352b0dcb1") ; [220]
  (ConceptNode "man") ; [221]
) ; [222]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (ConceptNode "Socrates@f250f429-5078-4453-8662-c63cc8f58a22") ; [217]
  (ConceptNode "man@80d4e852-1b93-4c49-84e1-5a7352b0dcb1") ; [220]
) ; [223]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (PredicateNode "is@9fb63f23-52ce-44ab-9bc4-806f0c4926e4") ; [224]
  (PredicateNode "be") ; [225]
) ; [226]

(InheritanceLink (stv .99 .99) (av 1000 0 0)
  (PredicateNode "is@9fb63f23-52ce-44ab-9bc4-806f0c4926e4") ; [224]
  (ConceptNode "present") ; [227]
) ; [228]

(PredicateNode "breathe@218b15b2-52a8-430c-93f6-b4bba225418c" (stv .001 .99)) ; [287]

(PredicateNode "breathe" (stv .001 .99)) ; [288]

(PredicateNode "is@9fb63f23-52ce-44ab-9bc4-806f0c4926e4" (stv .001 .99)) ; [224]

(PredicateNode "be" (stv .001 .99)) ; [225]
