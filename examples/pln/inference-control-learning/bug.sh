for i in {1..500}; do
    echo "Run $i"
    rm opencog.log &> /dev/null; guile -l bug.scm
    if [[ $? != 0 ]]; then
        exit 1
    fi
done
