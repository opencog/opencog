#!/bin/sh
if [ -z "$2" ]; then 
    echo "This script prints in real-time the evaluation result by the Rule Engine of any given precondition given in argument (actually only the prefix of a precondition is enough)"
    echo "Requires DEBUG or finer log level. Note that if you want a log when the evaluation returns false you need FINE log level"
    echo "usage: $0 precondition_schema_name OAC_LOG_FILE"
    exit
fi

str=" - RuleEngine - Precondition '""$1"

./tailgrep.sh $2 "$str"
