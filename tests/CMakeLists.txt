# Make sure testing is enabled
INCLUDE(MacroAddCxxTest)

# Use Python interpreter
ADD_SUBDIRECTORY (AtomSpace)
ADD_SUBDIRECTORY (NMXml)
ADD_SUBDIRECTORY (Util)
ADD_SUBDIRECTORY (CogServer)
ADD_SUBDIRECTORY (dynamics)
