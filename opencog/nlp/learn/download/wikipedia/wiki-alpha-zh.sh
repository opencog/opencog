#! /bin/bash
#
# Create subdirectories for alphabetized wikipedia articles,
# Move articles from main directory into subdirs.
# (This is the CJK version).
#
# Copyright (c) 2008, 2013, 2017 Linas Vepstas <linasvepstas@gmail.com>

mkdir A-Z
mkdir num
mkdir 一-亞
mkdir 亠-凌
mkdir 凍-十
mkdir 千-商
mkdir 問-大
mkdir 天-小
mkdir 少-廬
mkdir 廴-摜
mkdir 摩-朱
mkdir 朴-歡
mkdir 止-灣
mkdir 火-瓊
mkdir 瓜-籮
mkdir 米-羅
mkdir 羊-荷
mkdir 莆-觀
mkdir 角-重
mkdir 野-隴
mkdir 隶-香
mkdir 馬-龠
mkdir misc

# Must use find to do this, since "mv dir/A* otherdir/A"
# leads to an overflow on the shell command line.
echo "start A-Z"
time find ../wiki-stripped -name '[A-Z]*' -exec mv {} A-Z \;
time find ../wiki-stripped -name '[0-9]*' -exec mv {} num \;
echo "start 一"
time find ../wiki-stripped -name '[一-亞]*' -exec mv {} 一-亞 \;
time find ../wiki-stripped -name '[亠-凌]*' -exec mv {} 亠-凌 \;
time find ../wiki-stripped -name '[凍-十]*' -exec mv {} 	凍-十 \;
time find ../wiki-stripped -name '[千-商]*' -exec mv {} 	千-商 \;
time find ../wiki-stripped -name '[問-大]*' -exec mv {} 	問-大 \;
echo "start 二"
time find ../wiki-stripped -name '[天-小]*' -exec mv {} 	天-小 \;
time find ../wiki-stripped -name '[少-廬]*' -exec mv {} 	少-廬 \;
time find ../wiki-stripped -name '[廴-摜]*' -exec mv {} 	廴-摜 \;
time find ../wiki-stripped -name '[摩-朱]*' -exec mv {} 	摩-朱 \;
time find ../wiki-stripped -name '[朴-歡]*' -exec mv {} 	朴-歡 \;
echo "start 三"
time find ../wiki-stripped -name '[止-灣]*' -exec mv {} 	止-灣 \;
time find ../wiki-stripped -name '[火-瓊]*' -exec mv {} 	火-瓊 \;
time find ../wiki-stripped -name '[瓜-籮]*' -exec mv {} 	瓜-籮 \;
time find ../wiki-stripped -name '[米-羅]*' -exec mv {} 	米-羅 \;
time find ../wiki-stripped -name '[羊-荷]*' -exec mv {} 	羊-荷 \;
echo "start 四"
time find ../wiki-stripped -name '[莆-觀]*' -exec mv {} 	莆-觀 \;
time find ../wiki-stripped -name '[角-重]*' -exec mv {} 	角-重 \;
time find ../wiki-stripped -name '[野-隴]*' -exec mv {} 	野-隴 \;
time find ../wiki-stripped -name '[隶-香]*' -exec mv {} 	隶-香 \;
time find ../wiki-stripped -name '[馬-龠]*' -exec mv {} 	馬-龠 \;
time find ../wiki-stripped -name '*' -type f -exec mv {} misc \;

# Print out article counts.
echo "Article counts, by subdirectory"
echo -n A-Z ; find A-Z | wc
echo -n num ; find num | wc
echo -n 一-亞 ; find 一-亞 | wc
echo -n 亠-凌 ; find 亠-凌 | wc
echo -n 凍-十 ; find 凍-十 | wc
echo -n 千-商 ; find 千-商 | wc
echo -n 問-大 ; find 問-大 | wc
echo -n 天-小 ; find 天-小 | wc
echo -n 少-廬 ; find 少-廬 | wc
echo -n 廴-摜 ; find 廴-摜 | wc
echo -n 摩-朱 ; find 摩-朱 | wc
echo -n 朴-歡 ; find 朴-歡 | wc
echo -n 止-灣 ; find 止-灣 | wc
echo -n 火-瓊 ; find 火-瓊 | wc
echo -n 瓜-籮 ; find 瓜-籮 | wc
echo -n 米-羅 ; find 米-羅 | wc
echo -n 羊-荷 ; find 羊-荷 | wc
echo -n 莆-觀 ; find 莆-觀 | wc
echo -n 角-重 ; find 角-重 | wc
echo -n 野-隴 ; find 野-隴 | wc
echo -n 隶-香 ; find 隶-香 | wc
echo -n 馬-龠 ; find 馬-龠 | wc
echo -n misc ; find misc | wc
