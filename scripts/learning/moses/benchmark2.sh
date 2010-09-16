#!/bin/bash
STD=0 # standard reductor
SUPER=0 # super reductor
IGN_BSC=0 # ignore bscore
IGN_MOSHE=0 # ignore Moshe's tweak to ignore visisted candidates in
            # nondomination calculation
for i in `seq 1 100`;
do
    echo "Iteration $i"
    EVALS_STR=`moses-exec -H pa -k3 -a hc -r"$i" -V1 -m 500000 -P1 | grep "#evals:"`
    EVALS=${EVALS_STR#"#evals:"}
    STD=$(($EVALS+$STD))
    echo "Std Reduct: Evals = $EVALS, Total Evals = $STD"
    EVALS_STR=`moses-exec -H pa -k3 -a hc -r"$i" -V1 -m 500000 -P1 -E3 | grep "#evals:"`
    EVALS=${EVALS_STR#"#evals:"}
    SUPER=$(($EVALS+$SUPER))
    echo "Super Reduct: Evals = $EVALS, Total Evals = $SUPER"
    EVALS_STR=`moses-exec -H pa -k3 -a hc -r"$i" -V1 -m 500000 -P1 -I1 | grep "#evals:"`
    EVALS=${EVALS_STR#"#evals:"}
    IGN_BSC=$(($EVALS+$IGN_BSC))
    echo "Ignore Bscore: Evals = $EVALS, Total Evals = $IGN_BSC"
    EVALS_STR=`moses-exec -H pa -k3 -a hc -r"$i" -V1 -m 500000 -P1 -S0 | grep "#evals:"`
    EVALS=${EVALS_STR#"#evals:"}
    IGN_MOSHE=$(($EVALS+$IGN_MOSHE))
    echo "Ignore Moshe: Evals = $EVALS, Total Evals = $IGN_MOSHE"
done
STD_MEAN=$(($STD/100))
SUPER_MEAN=$(($SUPER/100))
IGN_BSC_MEAN=$(($IGN_BSC/100))
IGN_MOSHE_MEAN=$(($IGN_MOSHE/100))
echo "Std Reduct Mean Evals = $STD_MEAN"
echo "Super Reduct Mean Evals = $SUPER_MEAN"
echo "Ignore Bscore Mean Evals = $IGN_BSC_MEAN"
echo "Ignore Moshe Tweak Mean Evals = $IGN_MOSHE_MEAN"
