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
	cogutil 
)

