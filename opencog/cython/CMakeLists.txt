#
ADD_SUBDIRECTORY (opencog)

# direct file and path creation for stuff that doesn't need to be compiled
file(MAKE_DIRECTORY opencog)
# module init
file(COPY opencog/__init__.py DESTINATION opencog)
##
