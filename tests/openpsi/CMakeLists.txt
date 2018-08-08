LINK_LIBRARIES(
  ${ATOMSPACE_LIBRARIES}
  ${COGUTIL_LIBRARY}
  openpsi
)

# The tests are ordered in the order they are run during make test.
ADD_CXXTEST(OpenPsiRulesUTest)
ADD_CXXTEST(OpenPsiImplicatorUTest)
ADD_CXXTEST(OpenPsiSCMUTest)
