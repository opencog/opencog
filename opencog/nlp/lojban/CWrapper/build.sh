BUILD_DIR=$1

ghcver="$(stack ghc -- --version)"

if [[ $ghcver == *8.0.2* ]]
then
    echo "Correct GHC version installed"
else
    echo "Wrong GHC version installed. Running stack setup."
    stack setup
fi

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib/opencog"

libname=$(stack query --allow-different-user | awk 'NR==8' | sed 's/://g'| sed 's/ //g')
libver=$(stack query --allow-different-user | awk 'NR==10' | sed 's/version: //g' | sed "s/'//g" | sed "s/ //g")


if [ "$(id -u)" -ne 0 ];
then

    # Build haskell bindings package.
    stack build --allow-different-user
fi

LIB=$(find . -name "*$libname*.so" | awk 'NR==1')

patchelf --set-soname "lib$libname-$libver.so" $LIB

cp $LIB "$BUILD_DIR/lib$libname-$libver.so"
