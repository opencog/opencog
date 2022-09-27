#!/bin/bash
rm -f loader.scm
cd ..
for var in `ls rules/*.scm`
do
echo "(include \""$var"\")" >> "loader/loader.scm"
done
