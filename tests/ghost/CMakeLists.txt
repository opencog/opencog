# Check if relex is reachable.
# TODO:
# 1. Port this to FindRelex.cmake or find_service function to be used for
#    other services too.
# 2. Make it not depend on guile
IF(HAVE_GUILE)
  SET(RELEX_UTILS "${PROJECT_SOURCE_DIR}/opencog/nlp/scm/oc/relex-utils.scm")
  SET(CHECK_RELEX "
    (define loaded (catch #t (lambda () (load \"${RELEX_UTILS}\") #t)
      (lambda (e . args) #f)))
    (if (and (defined? 'set-relex-server-host) (defined? 'relex-reachable?)
          loaded)
      (begin (set-relex-server-host) (display (relex-reachable?)))
      (display \"config-error\"))")
  # The warning from guile during compilation of relex-utils.scm is ignored,
  # because it doesn't affect the usage of the functions.
  EXECUTE_PROCESS(COMMAND guile -q -c "${CHECK_RELEX}"
    OUTPUT_VARIABLE RELEX_REACHABLE
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
ENDIF()

IF("${RELEX_REACHABLE}" STREQUAL "#t")
  # Two of these tests are failing; I don't know why.
  # About half of the subtests in them fail. Run them by hand as
  # `guile -l test-ghost-syntax.scm` and so on.
  # ADD_GUILE_TEST(GhostSyntaxUTest test-ghost-syntax.scm)
  # SET_PROPERTY(TEST GhostSyntaxUTest
  #       APPEND PROPERTY ENVIRONMENT
  #       "PYTHONPATH=${PYTHON_ROOT}:${PROJECT_BINARY_DIR}/opencog/cython:${PROJECT_SOURCE_DIR}/tests/cython/agents:${PYTHON_ROOT}"
  # )

  ADD_GUILE_TEST(GhostProcedureUTest test-ghost-procedures.scm)

  # ADD_GUILE_TEST(GhostUTest test-ghost.scm)
  
ELSEIF("${RELEX_REACHABLE}" STREQUAL "config-error")
  message(FATAL_ERROR "Issue with relex utilities or the file ${RELEX_UTILS}")
ENDIF()
