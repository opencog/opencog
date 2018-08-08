SET(JSON_SPIRIT_SRCS
	json_spirit_reader.cpp json_spirit_reader.h
	json_spirit_value.cpp json_spirit_value.h
	json_spirit_writer.cpp json_spirit_writer.h
	json_spirit.h
	json_spirit_error_position.h
	json_spirit_reader_template.h
	json_spirit_stream_reader.h
	json_spirit_utils.h
	json_spirit_writer_options.h
	json_spirit_writer_template.h
)

ADD_LIBRARY(json_spirit SHARED ${JSON_SPIRIT_SRCS})

TARGET_LINK_LIBRARIES (json_spirit
	${Boost_THREAD_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
)
