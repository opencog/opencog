#!/usr/bin/bash

git clone



sudo apt-get install libboost-dev libboost-date-time-dev libboost-filesystem-dev \
libboost-program-options-dev libboost-regex-dev libboost-serialization-dev libboost-system-dev libboost-thread-dev;
sudo apt-get install cmake;
sudo apt-get install binutils-dev;
sudo apt-get install libiberty-dev;
sudo apt-get install doxygen;

#installation should happen in this order:
#cogutil
cd ~/opencog/cogutils;

mkdir build;
cd build;
cmake ..;
make -j$(nproc);
make test;
sudo make install;

#atomspace
cd ~/opencog/atomspace;

sudo apt-get install guile-2.0-dev;
sudo apt-get install cython;
sudo apt-get install haskell-platform;
sudo apt-get install postgresql postgresql-client;
sudo apt-get install libzmq3-dev; 
sudo apt-get install libprotobuf-dev; 

mkdir build;
cd build;
cmake ..;
make -j$(nproc);
make test;
sudo make install;

#moses(optional but recommended)
cd ~/opencog/moses;

sudo apt-get install libopenmpi-dev;
mkdir build;
cd build;
cmake -DCMAKE_BUILD_TYPE=Release ..;
make -j$(nproc);
sudo make install;

cd ~/opencog/atomspace;
mkdir build;
cd build;
cmake -DCMAKE_BUILD_TYPE=Release ..;
make -j$(nproc);
make test;
sudo make install;

sudo apt-get install liboctomap-dev;


#actual opencog
cd ~/opencog;
mkdir build;
cd build;
cmake ..;
make -j$(nproc); 
make test;



#persistance (for now; decentralized atomspace coming later)
sudo apt-get install libpq-dev;
cd ~/opencog/atomspace/opencog/persist/sql;


#relex
cd ~/opencog/relex;
install-scripts/install-ubuntu-dependencies.sh;
ant;
./relation-extractor.sh;



sudo pip freeze --local | grep -v '^\-e' | cut -d = -f 1  | xargs -n1 sudo -H pip install -U


#external tools
#SUMO importer
sudo -H pip install pythonds;
cd ~/opencog/external-tools/SUMO_importer;
bash sumo-opencog.sh;


cd ~/the_machine/opencog/external-tools/cnet_importer; wget --show-progress --continue https://github.com/opencog/test-datasets/releases/download/current/conceptnet5.4-scm-Jun-20-2016.tar.bz2


#python
sudo -H pip install nose;



