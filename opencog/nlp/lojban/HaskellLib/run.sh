if [ "$1" == "t" ] ; then
    stack exec lojban-test
else
echo "Starting Chat Bot"
    rlwrap stack exec lojbanChatBot
fi
