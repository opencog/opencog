mkdir -pv ~/opencog/docker/opencog
mkdir -pv ~/opencog/docker/distcc/opencog
mkdir -pv ~/opencog/docker/buildslave/opencog
mkdir -pv ~/opencog/docker/manualbuild/opencog
mkdir -pv ~/opencog/docker/embodiment/opencog
#echo opencog > ~/opencog/docker/.gitignore #in git
#echo opencog > ~/opencog/docker/distcc/.gitignore #in git
#echo opencog > ~/opencog/docker/buildslave/.gitignore #in git
#echo opencog > ~/opencog/docker/manualbuild/.gitignore #in git
#echo opencog > ~/opencog/docker/embodiment/.gitignore #in git
sudo mount -v --bind ~/opencog ~/opencog/docker/opencog
#sudo mount -v --bind ~/opencog ~/opencog/docker/distcc/opencog #not needed
sudo mount -v --bind ~/opencog ~/opencog/docker/buildslave/opencog
sudo mount -v --bind ~/opencog ~/opencog/docker/manualbuild/opencog
sudo mount -v --bind ~/opencog ~/opencog/docker/embodiment/opencog
