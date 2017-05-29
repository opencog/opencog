#
# Download and prep a bunch of texts from project gutenberg,
# and prep them for language-learning for English.  These titles
# comprise most of the "tranche-1" series of parsed texts.

wget http://www.gutenberg.org/files/76/76-0.txt
./chapters.sh 76-0.txt huck-finn-

scp huck-finn-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv huck-finn-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/5200.txt.utf-8
./chapters.sh 5200.txt.utf-8 kafka-

scp kafka-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv kafka-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/1661.txt.utf-8
./chapters.sh 1661.txt.utf-8 sherlock-

scp sherlock-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv sherlock-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/2701/2701-0.txt
./chapters.sh 2701-0.txt moby-dick-

scp moby-dick-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv moby-dick-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/345.txt.utf-8
./chapters.sh 345.txt.utf-8 dracula-

scp dracula-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv dracula-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/2591/2591-0.txt
./chapters.sh 2591-0.txt grimm-

scp grimm-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv grimm-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/1400/1400-0.txt
./chapters.sh 1400-0.txt expect-

scp expect-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv expect-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/158/158-0.txt
./chapters.sh 158-0.txt emma-

scp emma-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv emma-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/30254.txt.utf-8
./chapters.sh 30254.txt.utf-8 lust-

scp lust-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv lust-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/28054/28054-0.txt
./chapters.sh 28054-0.txt karamozov-

scp karamozov-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv karamozov-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/408.txt.utf-8
./chapters.sh 408.txt.utf-8 dubois-

scp dubois-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv dubois-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/20203.txt.utf-8
./chapters.sh 20203.txt.utf-8 franklin-

scp franklin-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv franklin-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/203/203-0.txt
./chapters.sh 203-0.txt uncle-tom-

scp uncle-tom-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv uncle-tom-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/833.txt.utf-8
./chapters.sh 833.txt.utf-8 veblen-

scp veblen-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv veblen-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/140/140-0.txt
./chapters.sh 140-0.txt jungle-

scp jungle-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv jungle-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/145.txt.utf-8
./chapters.sh 145.txt.utf-8 middlemarch-

scp middlemarch-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv middlemarch-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/54585/54585-0.txt
./chapters.sh 54585-0.txt venus-

scp venus-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv venus-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/1524.txt.utf-8
./chapters.sh 1524.txt.utf-8 hamlet-

scp hamlet-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv hamlet-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/21279.txt.utf-8
./chapters.sh 21279.txt.utf-8 vonn-

scp vonn-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv vonn-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/7142.txt.utf-8
./chapters.sh 7142.txt.utf-8 pelop-

scp pelop-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv pelop-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/18269/18269-0.txt
./chapters.sh 18269-0.txt pascal-

scp pascal-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv pascal-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/155/155-0.txt
./chapters.sh 155-0.txt moonstone-

scp moonstone-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv moonstone-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/files/54591/54591-0.txt
./chapters.sh 54591-0.txt japan-

scp japan-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv japan-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/files/5225/5225-0.txt
./chapters.sh 5225-0.txt satyr-

scp satyr-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv satyr-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/files/54579/54579-0.txt
./chapters.sh 54579-0.txt aero-

scp aero-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv aero-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/files/54613/54613-0.txt
./chapters.sh 54613-0.txt khe-

scp khe-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv khe-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/45631.txt.utf-8
./chapters.sh 45631.txt.utf-8 slave-

scp slave-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv slave-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/22657.txt.utf-8
./chapters.sh 22657.txt.utf-8 steam-

scp steam-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv steam-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/8164.txt.utf-8
./chapters.sh 8164.txt.utf-8 jeeves-

scp jeeves-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv jeeves-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/940/940-0.txt
./chapters.sh 940-0.txt mohicans-

scp mohicans-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv mohicans-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/31547.txt.utf-8
./chapters.sh 31547.txt.utf-8 youth-

scp youth-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv youth-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/851.txt.utf-8
./chapters.sh 851.txt.utf-8 rowland-

scp rowland-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv rowland-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/7452.txt.utf-8
./chapters.sh 7452.txt.utf-8 yogi-

scp yogi-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv yogi-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/54545/54545-0.txt
./chapters.sh 54545-0.txt century-

scp century-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv century-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/54634/54634-0.txt
./chapters.sh 54634-0.txt majorca-

scp majorca-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv majorca-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/2166.txt.utf-8
./chapters.sh 2166.txt.utf-8 mines-

scp mines-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv mines-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/2009.txt.utf-8
./chapters.sh 2009.txt.utf-8 darwin-

scp darwin-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv darwin-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/files/208/208-0.txt
./chapters.sh 208-0.txt daisy-

scp daisy-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv daisy-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/54665.txt.utf-8
./chapters.sh 54665.txt.utf-8 psychic-

scp psychic-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv psychic-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/599.txt.utf-8
./chapters.sh 599.txt.utf-8 vanity-

scp vanity-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv vanity-* split-books
mv *txt whole-books

wget http://www.gutenberg.org/ebooks/11224.txt.utf-8
./chapters.sh 11224.txt.utf-8 util-

scp util-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv util-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/15210.txt.utf-8
./chapters.sh 15210.txt.utf-8 darkwater-

scp darkwater-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv darkwater-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/16769/16769-0.txt
./chapters.sh 16769-0.txt orthodoxy-

scp orthodoxy-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv orthodoxy-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/54710/54710-0.txt
./chapters.sh 54710-0.txt shame-

scp shame-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv shame-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/621.txt.utf-8
./chapters.sh 621.txt.utf-8 relig-

scp relig-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv relig-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/8438.txt.utf-8
./chapters.sh 8438.txt.utf-8 ethics-

scp ethics-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv ethics-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/2892/2892-0.txt
./chapters.sh 2892-0.txt irish-

scp irish-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv irish-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/6519.txt.utf-8
./chapters.sh 6519.txt.utf-8 kabir-

scp kabir-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv kabir-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/29558.txt.utf-8
./chapters.sh 29558.txt.utf-8 scouts-

scp scouts-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv scouts-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/5720.txt.utf-8
./chapters.sh 5720.txt.utf-8 shrop-

scp shrop-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv shrop-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/ebooks/45502.txt.utf-8
./chapters.sh 45502.txt.utf-8 other-half-

scp other-half-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv other-half-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/8147/8147-0.txt
./chapters.sh 8147-0.txt kip-king-

scp kip-king-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv kip-king-* split-books
mv *utf-8 whole-books

wget http://www.gutenberg.org/files/54712/54712-0.txt
./chapters.sh 54712-0.txt penny-

scp penny-* ubuntu@10.0.3.208:/home/ubuntu/run/beta-guten
mv penny-* split-books
mv *txt whole-books

