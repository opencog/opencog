INSTALL (FILES
	post-processing.scm
	rule-helpers.scm
	utilities.scm
	DESTINATION "${DATADIR}/scm/nlp/relex2logic"
)

INSTALL (FILES
advmod.scm
amod.scm
atTime.scm
because.scm
be-inheritance.scm
compmod.scm
comp.scm
copula-ynq.scm
definite.scm
demdet.scm
howdeg-q.scm
howpredadj1-q.scm
how-q.scm
howquant-q.scm
IMPERATIVE.scm
neg.scm
poss.scm
pp.scm
pred-ynq.scm
prepadj.scm
PREP.scm
quantity.scm
rel.scm
rep.scm
SP.scm
SVIO1.scm
SVIO2.scm
svo.scm
SV.scm
TOBE.scm
todo1.scm
todo2.scm
todo5.scm
when-cop-q.scm
when-q.scm
where-cop-q.scm
where-q.scm
whichiobjQ.scm
whichobjQ.scm
whichobjSVIOQ.scm
whichpobjQ.scm
whichpredadjQ.scm
whichsubjpobjQ.scm
whichsubjQ.scm
whichsubjSVIOQ.scm
whichsubjSVOQ.scm
whichsubjSVQ.scm
why-cop-q.scm
why-q.scm
DESTINATION "${DATADIR}//scm/nlp/relex2logic"
)
ADD_SUBDIRECTORY (loader)
