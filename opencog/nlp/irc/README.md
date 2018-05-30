
IRC Chatbot interfaces
----------------------
Linas Vepstas <linasvepstas@gmail.com>

August 2009

Simple code that allows the OpenCog chatbot to connect to IRC.

Configure and run the IRC chat bridge:
--------------------------------------
The chatbot itself is just a simple deamon that bridges text from the
chat system to the OpenCog server. The cogita chatbot is tailored for
IRC chat; other chat systems could be supported in a straightforward
manner.

The IRC chatbot has an assortment of hard-coded config parameters in the
source code.  See `CogitaConfig.cc` for settable values for the name of
the chatbot, the irc channel and irc network to join, and the bot ID
strings, etc.  By default, it connects to the `#opencog` channel on
`freenode.net`.

The bot tries to connect to an opencog server at port 17004. This port
number is hard-coded in `whirr-sockets.cc`.

After modifying the hard-coded config as desired, start the bot by
saying `opencog/nlp/irc/cogita` in the build directory. It should then
appear on the IRC channel. You can then talk to it by addressing it by
name, i.e.  prefacing comments with `cog:`, `cogita:` or `cogita-bot:`
or by opening a private dialog window to it, in which case, you don't
need to address it directly.

Running the cogserver
---------------------
The cogserver can be started from guile, like so:
```
$ guile
scheme@(guile-user)> (use-modules (opencog)(opencog cogserver))
scheme@(guile-user)> (use-modules (opencog nlp) (opencog nlp chatbot))
scheme@(guile-user)> (start-cogserver "/path/to/opencog-git/lib/opencog-chatbot.conf")
```
This can be tested locally, without using IRC:
```
scheme@(guile-user)> (process-query "foo" "hello")
```


Architecture
-------------
IRC I/O, performed by cogita, acts as an intermediary between
IRC and OpenCog.  It listens for input on an IRC channel, and
forwards the resulting plain-text to OpenCog. This is done by
issuing one simple scheme expression to OpenCog, via the OpenCog
command-line interface/scheme shell on port 17004. The expression
is `(process-query user text)`, where the `user` is
the user's IRC nick, and `text` is what the user entered.  The
return value from this command is sent back to the IRC channel.
Communications is stateless and blocking: cogita closes the socket
to indicate end-of-messsage, and expects that the cog server will
do the same. Only after the cogserver closes its socket does cogita
reply on the IRC channel.

  * `IRC.cc,.h`:  C++ class for generic IRC communications.
  * `go-irc.cc`:  the main guts of the cogita server
  * `whirr-sockets.cc,.h`: tcp socket to send data to opencog, get reply.

Note that if the cogserver is busy, then `whirr` can block for an
indefinitely long time. The person who is chatting will start to
wonder about the lack of response. This is currently hacked around
by having the chat processing periodically return to the bridge,
and then resume again. A proper architecture remains unimplemented.
