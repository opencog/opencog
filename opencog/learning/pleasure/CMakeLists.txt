ADD_SUBDIRECTORY (pleasure)
ADD_SUBDIRECTORY (main)

ADD_LIBRARY (pleasure SHARED

	pleasure/generation
	pleasure/generation_table
	pleasure/node_list
	pleasure/population
	
)

TARGET_LINK_LIBRARIES (pleasure
        comboreduct
        util 
)


INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog
    ${OPENCOG_SOURCE_DIR}/opencog/learning
    ${OPENCOG_SOURCE_DIR}/opencog/learning/pleasure
)

LINK_DIRECTORIES (
    ${OPENCOG_BINARY_DIR}/opencog/learning/pleasure
    ${OPENCOG_BINARY_DIR}/opencog/comboreduct
    ${OPENCOG_BINARY_DIR}/opencog/util
)
