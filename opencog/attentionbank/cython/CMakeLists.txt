
ADD_SUBDIRECTORY (opencog)

# module init
file(MAKE_DIRECTORY opencog)
file(COPY opencog/__init__.py DESTINATION opencog)
