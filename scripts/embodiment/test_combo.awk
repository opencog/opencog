#!/usr/bin/gawk -f

BEGIN {
  N=200; # number of tests to run
  exec="../opencog/embodiment/Control/Procedure/combo_shell_stdio `find ../../ -name 'funcs.combo'`";
  FS="[ ()]";
#by elvys: 
#  Nowadys the functions names is not printed. So, I need to charge the
#  fucntions names from funcs.combo in cmd array bellow.
#  do {
#    exec ;
    exec |& getline;
#    print;
#    if ($(NF-1)=="0")
#      cmd[++n]=$(NF-2);
#  } while ($1!="loaded");
      cmd[++n]="seek_food_locally";
      cmd[++n]="sniff_butt";
      cmd[++n]="chase";
      cmd[++n]="pickup_carry_put_down";
      cmd[++n]="nudge_around";
      cmd[++n]="randbool";
      cmd[++n]="rand_pet_or_avatar";
      cmd[++n]="random_step";
      cmd[++n]="bark_non_friend_avatar";
      cmd[++n]="bark_non_friend_pet";
      cmd[++n]="lick_friendly_avatar";
      cmd[++n]="greet_avatar_with_jump";
      cmd[++n]="sleep_near_owner";
      cmd[++n]="sit_near_owner_and_stay";
      cmd[++n]="sit_near_someone_and_stay";

for (i=1;i<=N;++i) {
    todo=cmd[int(n*rand())+1];
    print todo |& exec;
    print todo;
    exec |& getline; 
    print;
    while (exec |& getline) {
      while ((NF>1 && $1==">" && $2=="exec:") || ($1=="exec:")) {
	print;
	exec |& getline;
      }	
      print; 
      if (/result:/) {
	exec |& getline;
	print;
	break;
      } else if (/true/) {
	if (rand()>0.5) {
	  print "true" |& exec;
	  print "true";
	} else {
	  print "false" |& exec;
	  print "false";
	}
      } else if (/sending/) {
	if (rand()>0.5) {
	  print "action_success" |& exec;
	  print "action_success";
	} else {
	  print "action_failure" |& exec;
	  print "action_failure";
	}
      } else if (/return/) {
	print "foo" |& exec;
	print "foo";
      } else {
	if (!/execution succeeded!/ &&
	    !/execution failed!/) {
	  print "Unrecognized output '"$0"'";
	  exit(1);
	}
	break;
      }
    }
  }
  print "Done! All "N" executions behaved themselves";
  exit(42);
}

