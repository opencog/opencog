#mkdir -v ~/opencog/docker/opencog #in git
#mkdir -v ~/opencog/docker/distcc/opencog #in git
#mkdir -v ~/opencog/docker/buildslave/opencog #in git
#echo opencog > ~/opencog/docker/.gitignore #in git
#echo opencog > ~/opencog/docker/distcc/.gitignore #in git
#echo opencog > ~/opencog/docker/buildslave/.gitignore #in git
sudo mount -v --bind ~/opencog ~/opencog/docker/opencog 
sudo mount -v --bind ~/opencog ~/opencog/docker/distcc/opencog
sudo mount -v --bind ~/opencog ~/opencog/docker/buildslave/opencog
