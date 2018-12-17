
INSTALL (FILES
	web/__init__.py
	DESTINATION "${PYTHON_DEST}/web")

INSTALL (FILES
	web/api/apiatomcollection.py
	web/api/apighost.py
	web/api/apimain.py
	web/api/apischeme.py
	web/api/apishell.py
	web/api/apitypes.py
	web/api/__init__.py
	web/api/mappers.py
	web/api/restapi.py
	web/api/utilities.py
	DESTINATION "${PYTHON_DEST}/web/api")

INSTALL (FILES
	web/socketio/atomspace_publisher.py
	web/socketio/__init__.py
	DESTINATION "${PYTHON_DEST}/web/socketio")
