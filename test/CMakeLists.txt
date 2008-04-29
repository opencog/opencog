# Make sure testing is enabled
ENABLE_TESTING()

INCLUDE(MacroAddCxxTest)

# Use Python interpreter
ADD_SUBDIRECTORY (AtomSpace)
ADD_SUBDIRECTORY (NMXml)
ADD_SUBDIRECTORY (Util)
ADD_SUBDIRECTORY (CogServer)
