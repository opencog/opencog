
#                             Example Agent

In OpenCog, an "Agent" is a bit of code that is run repeatedly, on a
regular schedule.  The CogServer maintains a list of active Agents, and
calls them on a periodic basis.  This directory provides an example of
how an agent can be written in C++.

To test this agent, do the following:

* Start the cogserver: From a shell prompt, issue the command
   `./opencog/cogserver/server/cogserver -c ../lib/development.conf` from the
   build directory

* Connect to the server: `telnet localhost 17001`

* At the cogserver prompt, issue the command:
  `opencog> loadmodule examples/agents/libexample_agent.so`
   You should see `done` printed on a line by itself.

* At the cogserver prompt, issue the command `listmodules`, and verify
   that the module was loaded. The module will be called
   `opencog::ExampleModule`, and so you should see:
   `Filename: libexample_agent.so, ID: opencog::ExampleModule`

* At the cogserver prompt, issue the command `agents-list`, and verify
   that the example agent is one of those available.  You should see the
   following printed:
   `opencog::ExampleAgent`

* At the cogserver prompt, issue the command `agents-active`, and verify
   that the example agent is running.  You should see the following
   printed:
   `opencog::ExampleAgent {""}`

   The agent was automatically started when the module was loaded.

* Stop the example agent. At the cogserver prompt, issue the command
   `agents-stop opencog::ExampleAgent`, and verify that the example
   agent stopped (by issuing `agents-active` again).

   Note that, at this time, with the current cogserver design, that
   "stopping an agent" means both stopping it, and calling its destructor.
   Starting the agent will call the constructor again (i.e. create a
   new instance of the agent.)

* Verify that the agent is running, or stopped, by looking at the
   `cogserver.log` file.  The example agent prints to the log file every
   time that it's run method is called.


That's all folks!  The rest is up to you!
