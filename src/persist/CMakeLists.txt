add_library (persist STATIC
odbc.cc
)

# I can't figure out how to use pkg-config with cmake, 
# so use a horrid hard-coded path instead :-(
# pkg-config --cflags glib-2.0
#
include_directories (
/usr/include/glib-2.0
/usr/lib/glib-2.0/include  
${OPENCOG_SOURCE_DIR}/src/AtomSpace
${OPENCOG_SOURCE_DIR}/src/Util
)

