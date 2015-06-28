#Now this is just a temporary workflow to test the pipeline from Minecraft to Opencog
#it's still not work now...
#Once I have time I would write a better version..

prerequisite:
1.put the classserver.pxd in atomspace/opencog/cython/opencog and build the atomspace
2.put the CMakeListspacetime.pyx/pxd, spatial.pyx/pxd, in opencog/opencog/cython/opencog and build the opencog

step to run:
1. start roscore and Minecraft Server
2. start cogserver, remember put this directory in the PYTHON_EXTENSION_DIRS variable in the opencog.conf so we can import those python file we need
3. run "echo -e 'py\n' | cat - opencog_intializer.py | netcat localhost 17001" to execute the initializer in cogserver
4. run mapnode.py and testbot3.py

For now(2015-06-28) the problem is that the mapnode.py doesn't put the block in the right location. And the cogserver would crush when call removeSpaceInfo, which shouldn't happened since the block doesn't change. I'm not sure the reason why it crashes, but I think it should be about the wrong location of block...