INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/opencog/atomspace
	${PROJECT_SOURCE_DIR}/opencog/persist/file
	${PROJECT_SOURCE_DIR}/opencog/persist/sql
	${PROJECT_SOURCE_DIR}/opencog/util
)

LINK_DIRECTORIES(
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/persist
	${PROJECT_BINARY_DIR}/opencog/util
)

LINK_LIBRARIES(
	atomspace
	persist
	server
	${Boost_SIGNALS_LIBRARY}
)

# The two tests below currrently work and pass, as of 10 December 2013.
# There's this complicated cmake junk, because they will fail for anyone who
# hasn't corrrectly configured their SQL setup.  To run these tests,
# see the README file, and re-run cmake.

FIND_PROGRAM(PSQL_EXECUTABLE psql)
IF (PSQL_EXECUTABLE)
	SET(ENV{PGPASSWORD} cheese)
	EXECUTE_PROCESS(COMMAND ${PSQL_EXECUTABLE} -c "SELECT * from TypeCodes;" -d test-persist -U opencog_tester -h localhost
		OUTPUT_VARIABLE OUT
		ERROR_VARIABLE ERR
		RESULT_VARIABLE RC)
	UNSET(ENV{PGPASSWORD})

	# Check the return code from PSQL
	IF (RC)
		MESSAGE(WARNING "${PSQL_EXECUTABLE}: ${ERR}")
	ELSE (RC)
		SET(DB_IS_CONFIGURED 1)
	ENDIF (RC)

ENDIF (PSQL_EXECUTABLE)

# If above doesn't work, then uncomment this line.
# SET(DB_IS_CONFIGURED 1)

IF (DB_IS_CONFIGURED)
	ADD_CXXTEST(BasicSaveUTest)
	ADD_CXXTEST(PersistUTest)
ELSE (DB_IS_CONFIGURED)
	MESSAGE(WARNING "SQL Database does not appear to be configured for unit tests!")
ENDIF (DB_IS_CONFIGURED)
