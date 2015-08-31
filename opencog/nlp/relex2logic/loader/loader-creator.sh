#!/bin/bash
rm load-rules.scm
cd ..
for var in `ls *.scm`
do
echo "(load-scm-from-file \"../opencog/nlp/relex2logic/"$var"\")" >>"loader/load-rules.scm"
done
echo "(load-scm-from-file \"../opencog/nlp/relex2logic/loader/gen-r2l-en-rulebase.scm\")">>"loader/load-rules.scm"
