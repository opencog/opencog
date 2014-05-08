Dockerfiles are static. Some developers use macro expansion via GNU make 
or other tools to make dockerfiles dyanic. For OpenCog's purposes, using
dockerfile inclusion should be sufficient. The Dockerfile at the git root
is self-contained and intended for simply running a cogserver in the most
common configuration, while the dockerfile here are designed to be built 
with inclusion according to the directory tree and for specific purposes.
