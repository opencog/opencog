

ADD_EXECUTABLE(serial_driver
	multi_vector
	pcg_multi
	serial_driver
)

TARGET_LINK_LIBRARIES(serial_driver
	blopex
)
