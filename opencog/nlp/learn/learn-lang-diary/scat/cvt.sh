#! /bin/bash

FLODIR=/home/linas/src/fractal/image/
BINDIR=/home/linas/src/fractal/generate/

# $BINDIR/takelog scat lscat
# $BINDIR/renorm lscat ren -0.2
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat.png

# $BINDIR/renorm scat ren 1.5
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-flat.png

# $BINDIR/renorm scat-cosine ren 1
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-cosine.png

# $BINDIR/renorm scat-cosine-big ren 1
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-cosine-big.png

$BINDIR/renorm scat-dcos ren 1
cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-dcos.png

# $BINDIR/renorm scat-ecos ren 1
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-ecos.png

# $BINDIR/renorm scat-ecos4 ren 1
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-ecos4.png

# $BINDIR/renorm scat-fcos ren 0.8
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-fcos.png

# $BINDIR/renorm scat-overlap ren 1.5
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-overlap.png

# $BINDIR/renorm scat-overlap-big ren 1.5
# cat ren.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-overlap-big.png

# cat /tmp/scat-cosmi.flo | $FLODIR/flo2mtv |mtvtoppm | pnmtopng > scat-pcos.png
