#!/bin/sh
if [ -e all_utests.log ]; then
	rm all_utests.log
fi
for utest in `find . -name "*UTest" ` ; do
	if [ -x $utest ]; then 
		printf "Running $utest  ";
		$utest > utest.log 2> /dev/null
		result=$?
		grep -Ei "Assertion failed|Error: Expected|Test failed|aborted|segmentation" utest.log  > /dev/null 2>&1
		if [  $? -eq 0 ] || [ $result -ne 0 ]; then
			echo "Failed";
			grep -Ei "Assertion failed|Test failed|aborted|segmentation" utest.log
			printf "\n\n"
			failed=true;
		else
			echo "OK"
		fi
	fi
        echo "$utest :" >> all_utests.log
        cat utest.log >> all_utests.log 
done

# test combo language
printf "Running ./scripts/test_combo.awk  ";
`find ../../ -name test_combo.awk` > utest.log 2> utest.log
result=$?
grep -Ei "Assertion failed|Test failed|aborted|segmentation" utest.log  > /dev/null 2>&1
if [  $? -eq 0 ] || [ $result -ne 42 ]; then
	echo "Failed";
	grep -Ei "Assertion failed|Test failed|aborted|segmentation" utest.log
	printf "\n\n"
	failed=true;
else
    echo "OK"
fi
echo "test_combo.awk :" >> all_utests.log
cat utest.log >> all_utests.log 

if [ $failed ]; then
	echo "SOME TESTS FAILED";
	exit -1
else
	echo "All tests successfull";
fi
