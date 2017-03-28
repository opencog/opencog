
# Hmm there does not seem to be any way of controlling the
# install location.  This needs fixing.
#ADD_GUILE_MODULE (chatbot-eva.scm
#	bot-api.scm
#	imperative.scm
#	imperative-rules.scm
#	knowledge.scm
#	model-query.scm
#	self-model.scm
#	semantics.scm
#)

INSTALL (FILES
	chatbot-eva.scm
	DESTINATION "${DATADIR}/scm/opencog/nlp")

INSTALL (FILES
	bot-api.scm
	imperative.scm
	imperative-rules.scm
	knowledge.scm
	load-sureal.scm
	model-query.scm
	self-model.scm
	semantics.scm
	DESTINATION "${DATADIR}/scm/opencog/nlp/chatbot-eva")
