#!/bin/sh
if [ -e system_tests.log ]; then
	rm system_tests.log
fi

SYSTEM_TEST_DIR=/tmp/$USER/SystemTest
rm -rf $SYSTEM_TEST_DIR

for gsfile in `find . -name "gsfile_*" ` ; do
	if [ -x $gsfile ]; then 

        ./killpb.csh
        ./pb.csh > /dev/null

        currfile=`echo $gsfile | awk -F/ '{print $2}'`
		printf "Running ./pbTester $currfile ... "
        ./pbTester $currfile
		result=$?

		grep -i "Error" /tmp/$USER/Petaverse/Logs/PROXY  > /dev/null 2>&1

		if [  $? -eq 0 ] || [ $result -eq 255 ]; then
			printf "Failed \n";
            
            dir_name=`echo $currfile | awk -F. '{print $1}'`
            LOGS_DIR=$SYSTEM_TEST_DIR/$dir_name
            mkdir -p $LOGS_DIR

            printf "Saving logs to $LOGS_DIR \n"
            cp -r /tmp/$USER/Petaverse/Logs $LOGS_DIR

			failed=true;
		else
			printf "OK\n"
		fi
	fi
done

./killpb.csh

if [ $failed ]; then
	echo "SOME TESTS FAILED";
	exit -1
else
	echo "All tests successfull";
    exit 0
fi
