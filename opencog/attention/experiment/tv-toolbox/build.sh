bin_dir="$1"
stack setup
stack build
echo "Haskell Build Script"
find . -name TVToolBoxCInterface_stub.h -exec mv {} "$bin_dir/." \;
find . -name "*TVToolBox*.so" -exec mv {} "$bin_dir/libjsd.so" \;
