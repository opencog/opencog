#! /bin/bash
#
# Create several diffeerent docker images for the current release of
# link-grammar. This script will download the latest release, and build
# containers for the parser, for the parse-server, and for the python
# bindings.
#
# Must be run as root, or use sudo.
#
# Build the base docker image. It just contains the link-grammar source.
cd docker-base
docker build --tag="linkgrammar/lgbase:latest" .

# Build the command-line parser
cd ../docker-parser
docker build --tag="linkgrammar/lgparser:latest" .

# Build and run the Java-based parse server.
cd ../docker-server
docker build --tag="linkgrammar/lgserver:latest" .

# Build the python bindings, run the unit tests.
cd ../docker-python
docker build --tag="linkgrammar/lgpython:latest" .
