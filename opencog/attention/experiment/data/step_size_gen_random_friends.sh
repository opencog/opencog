STEP_SIZE=5
SAMPLES=100
BASE_NAME="noise"

for i in {0..1000..10}
do
  ruby gen_random_friends.rb	 noise.scm $i
done
