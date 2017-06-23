#! /bin/bash

FLODIR=/home/linas/src/fractal/image/
BINDIR=/home/linas/src/fractal/generate/

# $BINDIR/takelog scat lscat
# $BINDIR/renorm lscat ren 0.02
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat.png

$BINDIR/renorm scat-foo ren 1
cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-flat.png

