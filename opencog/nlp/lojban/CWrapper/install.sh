#!/bin/bash

libname=$(stack query | awk 'NR==2' | sed 's/://g'| sed 's/ //g')
libver=$(stack query | awk 'NR==4' | sed 's/version: //g' | sed "s/'//g" | sed "s/ //g")

sudo rm /usr/local/lib/opencog/"lib$libname-$libver.so"

echo $libname

LIB=$(find . -name "*$libname*.so" | awk 'NR==1')

echo $LIB

echo $(ldd $LIB | grep 'not')

sudo mv $LIB "/usr/local/lib/opencog/lib$libname-$libver.so"
