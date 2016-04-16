bin_dir="$1"
stack setup
stack build
(>&2 echo "error")
find . -name TVToolBoxCInterface_stub.h -exec cp '{}' "$bin_dir/." ';'
find . -name "*TVToolBox*.so" -exec cp '{}' "$bin_dir/." ';'

