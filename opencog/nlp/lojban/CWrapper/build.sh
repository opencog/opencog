SOURCE_DIR=$1

libname=$(stack query | awk 'NR==5' | sed 's/://g'| sed 's/ //g')
libver=$(stack query | awk 'NR==7' | sed 's/version: //g' | sed "s/'//g" | sed "s/ //g")

if [ "$(id -u)" -ne 0 ]
then
    #Cleanup of last build if it exists
    rm -f "$SOURCE_DIR/lib$libname-$libver.so"

    # Build haskell bindings package.
    stack build

    LIB=$(find . -name "*$libname*.so" | awk 'NR==1')

    patchelf --set-soname "lib$libname-$libver.so" $LIB

    cp $LIB "$SOURCE_DIR/lib$libname-$libver.so"
else
    echo "Can't run Haskell-Tests as root"
fi
