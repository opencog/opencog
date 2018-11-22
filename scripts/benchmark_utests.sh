# Small script to test the performance of OpenCog. It merely runs all
# the unit tests a certain number of times and output statistics about
# it.

# This script relies on https://github.com/nferraz/st

# Check unbound variables
set -u

# # Debug trace
# set -x

# Number of times to run the unit tests
N=10

# Name of the build directory
BUILD_DIR_NAME=build

# Get the script directory
PRG_PATH="$(readlink -f "$0")"
PRG_DIR="$(dirname "$PRG_PATH")"
UTEST_DIR="$PRG_DIR/../$BUILD_DIR_NAME"

#############
# Functions #
#############

# Get the real time in second (given the output of time command)
get_real_time() {
    grep "real" | cut -d' ' -f2
}

# Run all query unit tests
run_utests() {
    cd "$UTEST_DIR";
    make -j4 test ARGS=-j4
    cd -
}

########
# Main #
########

for i in $(seq 1 $N); do
    echo "Run unit test suite ($i/$N)" 1>&2
    (time -p run_utests 1>/dev/null) 2>&1
done | get_real_time | st | column -t
