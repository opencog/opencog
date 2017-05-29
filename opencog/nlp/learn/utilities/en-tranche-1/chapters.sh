#! /bin/bash

# Split big project-gutenberrg files into parts.
# takes two arguments: the first argument is the filename to split,
# the second is the filename to generate.

# take file in argument 1, and replace all double-newlines
# by the control-K character.
cat $1 | sed ':a;N;$!ba;s/\n/xxx-foo-xxx/g' > xxx
cat xxx |sed 's/xxx-foo-xxx\rxxx-foo-xxx/\n\x0b\n/g' > yyy
cat yyy |sed 's/\rxxx-foo-xxx/\n/g' > zzz
# cat xxx |sed 's/xxx-foo-xxxxxx-foo-xxx/\n\x0b\n/g' > yyy
# cat yyy |sed 's/xxx-foo-xxx/\n/g' > zzz

# split the file along control-K into parts with 50 paragraphs each.
# split -t '' zzz poop-
split -l 50 -t '' --filter=' sed "s///g" > $FILE' zzz $2

# remove temps
rm xxx yyy zzz
