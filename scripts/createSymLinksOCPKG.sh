GITROOT=$1
BUILDROOT=$2

INITIAL_PWD=$PWD
SEARCH_PWD=$PWD

echo "Starting symbolic link creation script..."

if ! [ -e $GITROOT ]; then
	echo "Supplied GITROOT does not exist!"
	exit
fi

# In case no GITROOT was supplied...
if [ -z $GITROOT ]; then
	echo "Trying to locate git root..."

	PWDLEN=$(echo ${#PWD})
	
	# See if we can search upwards...
	if [ $PWDLEN -gt 1 ]; then
		# try to find it by going up the folder tree
		while [ $PWDLEN -gt 1 ] && [ -z $GITROOT ] ; do
			if [ -e ".git" ] && [ -e ${PWD}/lib/opencog.conf ]; then
				GITROOT=${PWD}
			else
				#echo "No .git file found, going up!"
				cd ..
				SEARCH_PWD=$PWD
				PWDLEN=$(echo ${#SEARCH_PWD})
			fi
		done
	fi

	# If we still have no GITROOT...search downwards...
	if [ -z $GITROOT ]; then
		echo "Unable to locate git root by going up..."
		echo "Returning to initial folder $INITIAL_PWD"
		echo "Trying to locate git root in subfolders..."
		cd $INITIAL_PWD
		# Check for ochack folder
		if [ -e "ochack" ]; then
			cd ochack
			if [ -e ".git" ] && [ -e ${PWD}/lib/opencog.conf ]; then
				GITROOT=${PWD}
			fi				
		# Check for opencog folder
		elif [ -e "opencog" ]; then
			cd opencog
			if [ -e ".git" ] && [ -e ${PWD}/lib/opencog.conf ]; then
				GITROOT=${PWD}
			else
				#check for a src folder here
				if [ -e "src" ]; then
					cd src
					if [ -e ".git" ]; then
						GITROOT=${PWD}
					fi
				fi
			fi				
		# Check for src folder
		elif [ -e "src" ]; then
			cd src
			if [ -e ".git" ] && [ -e ${PWD}/lib/opencog.conf ]; then
				GITROOT=${PWD}
			else
				#check for an opencog folder here
				if [ -e "opencog" ]; then
					cd opencog
					# check for a src folder here
					if [ -e "src" ]; then
						cd src
						if [ -e ".git" ] && [ -e ${PWD}/lib/opencog.conf ]; then
							GITROOT=${PWD}
						fi
					fi
				fi
			fi				
		fi
	fi
fi

if ! [ -z $GITROOT ]; then
	if ! [ -z $GITROOT ]; then
		# Let's go into GITROOT
		echo "Moving into $GITROOT"
		cd $GITROOT
		# Check for and delete existing bin folder
		if [ -e "bin" ]; then
			echo "Found an existing bin folder, deleting..."
			rm -rf bin
			echo "Existing bin folder deleted!"
		fi

		echo "Creating folders for symbolic links..."

		# Let's create the folders first
		mkdir -p bin/opencog/server
		mkdir -p bin/opencog/persist/sql
		mkdir -p bin/opencog/query
		mkdir -p bin/opencog/shell
		mkdir -p bin/opencog/web
		mkdir -p bin/opencog/visualization/ubigraph
		mkdir -p bin/opencog/learning/dimensionalembedding
		mkdir -p bin/opencog/cython
		mkdir -p bin/opencog/embodiment/AtomSpaceExtensions/
		mkdir -p bin/opencog/util
		mkdir -p bin/opencog/nlp/types
#		mkdir -p bin/opencog/viterbi
		mkdir -p bin/opencog/dynamics/attention
		mkdir -p bin/opencog/atomspace
		mkdir -p bin/opencog/spacetime

		echo "Folder creation complete!"

		# And then the symbolic links
		# BUILDROOT = /tmp/opencog/build
		# ln -s ../../../../build/opencog/server/cogserver bin/opencog/server/cogserver

		#echo "GITROOT=$GITROOT"			

		if [ -z $BUILDROOT ]; then
			BUILDROOT=$GITROOT/../build
		fi 

		if ! [ -e $BUILDROOT ]; then
			mkdir -p $BUILDROOT
		fi

		#echo "BUILDROOT=$BUILDROOT"			

		echo "Creating symbolic links..."
		ln -s $BUILDROOT/opencog/spacetime/spacetime_types.scm bin/opencog/spacetime/spacetime_types.scm

		ln -s $BUILDROOT/opencog/atomspace/core_types.scm bin/opencog/atomspace/core_types.scm

		ln -s $BUILDROOT/opencog/server/cogserver bin/opencog/server/cogserver
		ln -s $BUILDROOT/opencog/server/libbuiltinreqs.so bin/opencog/server/libbuiltinreqs.so
		ln -s $BUILDROOT/opencog/persist/sql/libpersist.so bin/opencog/persist/sql/libpersist.so
		ln -s $BUILDROOT/opencog/query/libquery.so bin/opencog/query/libquery.so
		ln -s $BUILDROOT/opencog/shell/libscheme-shell.so bin/opencog/shell/libscheme-shell.so
		ln -s $BUILDROOT/opencog/shell/libpy-shell.so bin/opencog/shell/libpy-shell.so
		ln -s $BUILDROOT/opencog/web/libocweb.so bin/opencog/web/libocweb.so

		ln -s $BUILDROOT/opencog/visualization/ubigraph/libubigraph.so bin/opencog/visualization/ubigraph/libubigraph.so

		ln -s $BUILDROOT/opencog/learning/dimensionalembedding/libdimensionalembedding.so bin/opencog/learning/dimensionalembedding/libdimensionalembedding.so

		ln -s $BUILDROOT/opencog/cython/libPythonModule.so bin/opencog/cython/libPythonModule.so

		ln -s $BUILDROOT/opencog/embodiment/AtomSpaceExtensions/libAtomSpaceExtensions.so bin/opencog/embodiment/AtomSpaceExtensions/libAtomSpaceExtensions.so
		ln -s $BUILDROOT/opencog/embodiment/AtomSpaceExtensions/embodiment_types.scm bin/opencog/embodiment/AtomSpaceExtensions/embodiment_types.scm
		ln -s $BUILDROOT/opencog/util/libcogutil.so bin/opencog/util/libcogutil.so

		ln -s $BUILDROOT/opencog/nlp/types/libnlp-types.so bin/opencog/nlp/types/libnlp-types.so
		ln -s $BUILDROOT/opencog/nlp/types/nlp_types.scm bin/opencog/nlp/types/nlp_types.scm

#		ln -s $BUILDROOT/opencog/viterbi/libviterbi.so bin/opencog/viterbi/libviterbi.so
		ln -s $BUILDROOT/opencog/dynamics/attention/libattention.so bin/opencog/dynamics/attention/libattention.so
		ln -s $BUILDROOT/opencog/dynamics/attention/attention_types.scm bin/opencog/dynamics/attention/attention_types.scm
#		ln -s $BUILDROOT/opencog/ubigraph/libubigraph.so bin/opencog/ubigraph/libubigraph.so

		echo "Symbolic link creation complete!"
	fi
else
	echo "No git root folder found, try running again from inside the opencog source tree"
fi 
