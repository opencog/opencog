Dockerfiles are static. Some developers use macro expansion via GNU make 
or other tools to make dockerfiles dynamic. For OpenCog's purposes, using
dockerfile inclusion should be sufficient. The Dockerfile at the git root
is self-contained and intended for simply running a cogserver in the most
common configuration, while the dockerfiles here are designed to be built 
in an additive way, all depending on the container tagged opencog-deps in
the same directory as this README.

Docker image structure:

    ├─opencog-deps
      ├─opencog-dev (for a dev environment)
      ├─opencog-buildslave
      ├─opencog-distcc
      ├─opencog-embodiment

Using bind mounts (some containers currently use them) is a hacky solution
to passing the opencog source tree to each docker build. Run bindmounts.sh
before running 'docker build' and bindumount.sh to clean up afterward.
