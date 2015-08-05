

                          Scripts README
                          --------------

Files here are meant to be somehow related to opencog, and yet not
central to its operation.

 * embodiment directory has embodiment related scripts. Note that the
   embodiment component is in the midst of being re-implemented and
   might not currently work.

 * run_chatbot_and_servers.sh - run the cogserver, the RelEx server, and the
   cogita chat-bot in the tabs of a gnome-terminal.


For Installing dependencies for this project and building/testing the project
use the octool associated with https://github.com/opencog/ocpkg/blob/master/ocpkg.
To use the script follow the following steps

1. wget -O octool https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg
2. chmod 755 octool
3. ./octool -rdcv     # install dependencies
4. ./octool -b        # build the code
5. ./octool -e        # build examples
6. ./octool -t        # build tests
