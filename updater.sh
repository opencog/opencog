#!/usr/bin/bash
echo "Pulling nnew stuff..."
cd ~/the_machine/opencog;
proxychains git pull upstream master;
pwd;
cd ~/the_machine/opencog/cogutil/; proxychains git pull upstream master;
pwd;
cd ~/the_machine/opencog/atomspace/; proxychains git pull upstream master;
pwd;
cd ~/the_machine/opencog/moses/; proxychains git pull upstream master;
pwd;
cd ~/the_machine/opencog/external-tools/; proxychains git pull;
pwd;
cd ~/the_machine/opencog/ghost_bridge/; proxychains git pull;
pwd;
cd ~/the_machine/opencog/language-learning/; proxychains git pull;
pwd;
cd ~/the_machine/opencog/as-moses/; proxychains git pull;
pwd;
cd ..;
echo "(Re)Building stuff using octool..."
./octool -camlb;
