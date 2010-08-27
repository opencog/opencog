#!/bin/bash
STD=0
SUPER=0
for i in `seq 1 100`;
do
    echo "Iteration $i"
    EVALS_STR=`moses-exec -H pa -k3 -a hc -r"$i" -V1 -m 500000 | grep "#evals:"`
    EVALS=${EVALS_STR#"#evals:"}
    STD=$(($EVALS+$STD))
    echo "Standard Reduct: Evals = $EVALS, Total Evals = $STD"
    EVALS_STR=`moses-exec -H pa -k3 -a hc -r"$i" -V1 -m 500000 -E3 | grep "#evals:"`
    EVALS=${EVALS_STR#"#evals:"}
    SUPER=$(($EVALS+$SUPER))
    echo "Super Reduct: Evals = $EVALS, Total Evals = $SUPER"
done

