#!/bin/bash

# Script test to attempt to load MOSES models in scheme format to the
# AtomSpace so that PLN can then reason on them.
#
# It performs the following
#
# 1. Launch an OpenCog server
#
# 2. Load background knowledge from a Scheme file (like feature
# definitions)
#
# 3. Run MOSES on some problem
#
# 4. Parse the output and pipe it in OpenCog
#
# 5. Use PLN to perform reasoning, etc.

set -u
set -x

if [[ $# != 1 ]]; then
    echo "Usage: $0 SETTINGS_FILE"
    exit 1
fi

#############
# Functions #
#############

# Given an error message, display that error on stderr and exit
fatalError() {
    echo "[ERROR] $@" 1>&2
    exit 1
}

warnEcho() {
    echo "[WARN] $@"    
}

infoEcho() {
    echo "[INFO] $@"
}

# Convert human readable integer into machine full integer. For
# instance $(hr2i 100K) returns 100000, $(hr2i 10M) returns 10000000.
hr2i() {
    local val=$1
    local val=${val/M/000K}
    local val=${val/K/000}
    echo $val
}

# pad $1 symbol with up to $2 0s
pad() {
    pad_expression="%0${2}d"
    printf "$pad_expression" "$1"
}

########
# Main #
########

# Source settings
. $1

# 1. Launch an OpenCog server

cd "$opencog_repo_path/scripts/"
./run_cogserver.sh "$build_dir_name" &
cd -
sleep 5

# 2. Load background knowledge

(echo "scm"; cat "$scheme_file_path") \
    | "$opencog_repo_path/scripts/run_telnet_cogserver.sh"

# 3. Run MOSES

moses_output_file=results.moses

moses \
    -i "$dataset_file_path" \
    --output-score 0 \
    --output-with-labels 1 \
    --output-format scheme \
    --jobs $jobs \
    --max-evals $(hr2i $evals) \
    --output-file $moses_output_file

# 4. Parse MOSES output and pipe it in OpenCog

# TODO: wrap the program with an EquivalentLink with model name
# EquivalentLink<1,1>
#     EvaluationLink
#         PredicateNode: "model XX"
#         VariableNode: "$X"
#     EvaluationLink
#         ...

(echo "scm";
    i=0
    while read line; do
        moses_model_name="moses_$(pad $i 3)"
        echo "(EquivalentLink (EvaluationLink (PredicateNode \"${moses_model_name}\" $line)))"
    done < "$moses_output_file"
) | "$opencog_repo_path/scripts/run_telnet_cogserver.sh"

# 5. Use PLN to perform reasoning, etc.
# TODO
