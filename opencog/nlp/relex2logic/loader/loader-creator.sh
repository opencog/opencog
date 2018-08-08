#!/bin/bash
rm load-rules.scm
cd ..
for var in `ls *.scm`
do
echo "(load \"../"$var"\")" >>"loader/load-rules.scm"
done
