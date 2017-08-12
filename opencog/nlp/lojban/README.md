## Steps to take
1. For building the code run the following commands from the build directory

   ```
   cmake -DBUILD_LOJBAN=1 ..
   make -j$(nproc)
   ```

2. Start the cogserver. It will need internet connection for downloading a file
   on the first run.

   ```
   ./opencog/cogserver/server/cogserver -c ../lib/lojban.conf
   ```

3. Connect to the opencog shell

   ```
   rlwrap telnet localhost 17001
   ```

4. Test the parser
   ```
   parse-lojban mi jimpe ti
   ```
##TODO
Bugs and open work items.

* Lojban currently defines a SetSizeLink and a SetTypeLink which are
  terribly generic names and are completely undocumented. Can we pick
  a better name for these things, so as to avoid confusion with
  SetLink (which is about sets) and the various Type Nodes and Links
  that handle opencog Atom types?
