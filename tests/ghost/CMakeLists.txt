# Check if relex is reachable.
IF(HAVE_GUILE)
  SET(CHECK_RELEX
    "(load \"${PROJECT_SOURCE_DIR}/opencog/nlp/scm/relex-utils.scm\")
     (set-relex-server-host) (display (relex-reachable?))")
  EXECUTE_PROCESS(COMMAND guile -c "${CHECK_RELEX}"
    OUTPUT_VARIABLE RELEX_REACHABLE
    OUTPUT_STRIP_TRAILING_WHITESPACE)
ENDIF()

IF("${RELEX_REACHABLE}" STREQUAL "#t")
  ADD_GUILE_TEST(GhostProcedureTest test-ghost-procedures.scm)
ENDIF()
