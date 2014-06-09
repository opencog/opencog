mkdir -pv ~/opencog/docker/opencog
mkdir -pv ~/opencog/docker/buildslave/opencog
mkdir -pv ~/opencog/docker/embodiment/opencog
sudo mount -v --bind ~/opencog ~/opencog/docker/opencog
sudo mount -v --bind ~/opencog ~/opencog/docker/buildslave/opencog
sudo mount -v --bind ~/opencog ~/opencog/docker/embodiment/opencog
