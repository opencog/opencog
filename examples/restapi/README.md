This directory contains example scripts for starting the restapi and
interacting with it through a client.

# Steps
1. Install atomspace and cogutils
2. Run the following, replace `/path/to/opencog/clone` with your actual git
   clone path.

   ```
   export PYTHONPATH="${PYTHONPATH}:/usr/local/share/opencog/python"
   export PYTHONPATH="${PYTHONPATH}:/path/to/opencog/clone/opencog/python/"
   export PYTHONPATH="${PYTHONPATH}:/path/to/opencog/clone/build/opencog/cython"
   ```

3. Start the restapi. There are two options use either one.
   * run `python start_restapi.py`
   * run `guile -l start-restapi.scm`
4. In a separate terminal run ` python exampleclient.py` for interacting with
   the atomspace.
