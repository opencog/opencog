if [ "$1" == "t" ] ; then
./.stack-work/install/x86_64-linux-nopie/lts-8.5/8.0.2/bin/lojban-test #+RTS -p
else
echo "Starting Chat Bot"
./.stack-work/install/x86_64-linux-nopie/lts-8.5/8.0.2/bin/lojbanChatBot #+RTS -xc -RTS
fi
