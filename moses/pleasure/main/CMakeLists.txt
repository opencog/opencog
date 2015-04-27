
ADD_EXECUTABLE (pleasure-main main.cpp)
TARGET_LINK_LIBRARIES (pleasure-main
	pleasure
	comboreduct
	comboant
	cogutil
)

ADD_CUSTOM_COMMAND(
	TARGET main
	POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy ${OPENCOG_SOURCE_DIR}/opencog/learning/pleasure/main/nodelist ${OPENCOG_BINARY_DIR}/opencog/learning/pleasure/main
)


