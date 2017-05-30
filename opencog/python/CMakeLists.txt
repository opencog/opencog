
#
# TODO - these should be in their own opencog module.
# XXX This is a really terrible naming convention, the web api
# should not be pretending that its something generic, and not
# merely opencog-specific. FIXME
INSTALL (FILES
	web/__init__.py
	DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/web")

INSTALL (FILES
	web/api/apiatomcollection.py
	web/api/apimain.py
	web/api/apischeme.py
	web/api/apishell.py
	web/api/apitypes.py
	web/api/__init__.py
	web/api/mappers.py
	web/api/restapi.py
	web/api/utilities.py
	DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/web/api")

INSTALL (FILES
	web/socketio/atomspace_publisher.py
	web/socketio/__init__.py
	DESTINATION "lib${LIB_DIR_SUFFIX}/python2.7/dist-packages/web/socketio")
