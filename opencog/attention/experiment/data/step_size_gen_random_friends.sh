STEP_SIZE=5
SAMPLES=100
BASE_NAME="noise"

for i in {0..100..5}
do
  ruby gen_random_friends.rb	 $BASE_NAME'-'$i'.scm' $i
done
