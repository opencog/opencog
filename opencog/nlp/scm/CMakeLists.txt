# NOTE: relex-utils.scm is used by cmake for configuring relex dependent tests.
# Update relevant paths should you move it.
ADD_GUILE_MODULE (FILES
	oc.scm
	oc/relex-utils.scm
	oc/processing-utils.scm
	MODULE_DESTINATION "${GUILE_SITE_DIR}/opencog/nlp/oc"
)
