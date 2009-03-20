ADD_LIBRARY(PBTester
	PBTester
	TestParameters
	GoldStdReaderTask
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/PerceptionActionInterface
	${OPENCOG_SOURCE_DIR}/include/xerces
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial
        ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Spatial/Math
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control/Predavese
	${OPENCOG_SOURCE_DIR}/opencog/embodiment/PetaverseProxySimulator
	${OPENCOG_SOURCE_DIR}/test/AutomatedSystemTest
)

LINK_DIRECTORIES (
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
	${OPENCOG_BINARY_DIR}/opencog/embodiment/Control/MessagingSystem
	${OPENCOG_SOURCE_DIR}/lib/linux/xerces
)


TARGET_LINK_LIBRARIES (PBTester PVPSimulator)

ADD_EXECUTABLE (pbTester PBTesterExecutable)

TARGET_LINK_LIBRARIES (pbTester PBTester)

