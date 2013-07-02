#!/bin/bash
h_exe=examples/hopfield/hopfield
results_dir=hopfield_results
if [ -d $results_dir ]
then
    echo "Replacing ${results_dir}, ctrl-c to abort, enter to continue..."
    read
    rm -R $results_dir
fi
mkdir ${results_dir}

densities="0.4 0.8 1.0"
focus_threshold=5
viz_threshold=5
error=0.2
stimulus=10
retrieval_cycles=10
flags="-R -E"

# test random created patterns
sizes="3 4 5"

npatterns=30
gen_density=0.3

echo "Testing random patterns"

for n in $sizes
do
    for d in $densities
    do
	echo "Size $n - density $d"
	logfile=log_n_${n}_d_${d}.txt
	options="-n $n -d $d -a results_n_${n}_d_${d}_ \
	    -f ${focus_threshold} -z ${viz_threshold} -e ${error} \
	    -c ${retrieval_cycles} -s ${stimulus} -v -v -v \
	    -g ${gen_density} -p ${npatterns} ${flags}"
	echo " === Command line:" > $logfile
	echo $options >> $logfile
	echo " === Output of -C from program:" >> $logfile
	$h_exe $options -C >> ${logfile}
	echo " ===" >> $logfile
	$h_exe $options >> ${logfile}
	mv ${logfile} ${results_dir}/.
	gzip ${results_dir}/${logfile} 
	mv results_n_${n}_d_${d}_* ${results_dir}/.

    done
done


# test user made patterns 
echo "---------------------"
echo "Testing user patterns"

densities="0.05 0.5 0.8"
n=10
train_file=../src/examples/hopfield/training_patterns.txt
cue_file=../src/examples/hopfield/cue_patterns.txt
results_file=${results_dir}/result_user_pattern_n_${n}_d_${d}.txt
for d in $densities
do
    logfile=log_user_pattern_n_${n}_d_${d}.txt
    echo "Size $n - density $d"
    options="-n $n -d $d -a results_n_${n}_d_${d}_ \
	-f ${focus_threshold} -z ${viz_threshold} -e ${error} \
	-c ${retrieval_cycles} -s ${stimulus} -v -v -v \
	--train-file=${train_file} \
	--result-file=${results_file} \
	--cue-file=${cue_file} ${flags}"
    echo " === Command line:" > $logfile
    echo $options >> $logfile
    echo " === Output of -C from program:" >> $logfile
    $h_exe $options -C >> ${logfile}
    echo " ===" >> $logfile
    $h_exe $options >> ${logfile}
    mv ${logfile} ${results_dir}/.
    gzip ${results_dir}/${logfile} 
    mv results_n_${n}_d_${d}_* ${results_dir}/.

done

#---
n=8
train_file=../src/examples/hopfield/training_patterns_simple.txt
results_file=${results_dir}/result_user_pattern_n_${n}_d_${d}.txt
for d in $densities
do
    logfile=log_user_pattern_n_${n}_d_${d}.txt
    echo "Size $n - density $d"
    options="-n $n -d $d -a results_n_${n}_d_${d}_ \
	-f ${focus_threshold} -z ${viz_threshold} -e ${error} \
	-c ${retrieval_cycles} -s ${stimulus} -v -v -v \
	--train-file=${train_file} \
	--result-file=${results_file} ${flags}"
    echo " === Command line:" > $logfile
    echo $options >> $logfile
    echo " === Output of -C from program:" >> $logfile
    $h_exe $options -C >> ${logfile}
    echo " ===" >> $logfile
    $h_exe $options >> ${logfile}
    mv ${logfile} ${results_dir}/.
    gzip ${results_dir}/${logfile} 
    mv results_n_${n}_d_${d}_* ${results_dir}/.

done
