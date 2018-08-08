#!/bin/bash

This is broken/wrong ...

rm gen-r2l-en-rulebase.scm
cd ..
echo "(InheritanceLink (stv 1 1) (ConceptNode \"R2L-en-RuleBase\") (ConceptNode \"RuleBase\"))">>"loader/gen-r2l-en-rulebase.scm"
echo "(define r2l-rules (ConceptNode \"R2L-en-RuleBase\"))">>"loader/gen-r2l-en-rulebase.scm"
echo "(ExecutionLink (SchemaNode \"URE:maximum-iterations\") (ConceptNode \"R2L-en-RuleBase\") (NumberNode \"100\") )">>"loader/gen-r2l-en-rulebase.scm"
for var in `ls *.scm`
do
echo "(define "$var"-name (Node \"$var\"))" >>"loader/gen-r2l-en-rulebase.scm"
echo "(DefineLink "$var"-name $var)" >>"loader/gen-r2l-en-rulebase.scm"
echo "(MemberLink (stv 1 1) "$var"-name (ConceptNode \"R2L-en-RuleBase\"))" >>"loader/gen-r2l-en-rulebase.scm"
done
