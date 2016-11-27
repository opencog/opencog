if [ $1 == "t" ] ; then
./.stack-work/install/x86_64-linux/lts-7.3/8.0.1/bin/lojban-test #+RTS -p
else
echo "Starting Chat Bot"
./.stack-work/install/x86_64-linux/lts-7.3/8.0.1/bin/lojbanChatBot #+RTS -xc -RTS
fi
