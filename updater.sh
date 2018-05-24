#!/usr/bin/bash
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
cd ..;