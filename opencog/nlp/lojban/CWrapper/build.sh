SOURCE_DIR=$1

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib/opencog"

libname=$(stack query --allow-different-user | awk 'NR==5' | sed 's/://g'| sed 's/ //g')
libver=$(stack query --allow-different-user | awk 'NR==7' | sed 's/version: //g' | sed "s/'//g" | sed "s/ //g")

#Cleanup of last build if it exists
rm -f "$SOURCE_DIR/lib$libname-$libver.so"

# Build haskell bindings package.
stack build --allow-different-user

LIB=$(find . -name "*$libname*.so" | awk 'NR==1')

patchelf --set-soname "lib$libname-$libver.so" $LIB

cp $LIB "$SOURCE_DIR/lib$libname-$libver.so"
