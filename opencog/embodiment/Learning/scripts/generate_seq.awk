#! /bin/awk -f
# to generate the code for N ctors for the sequential rule, do:
# generate_seq.awk N

function doit(x,i) {
  printf("sequential(");
  for (i=1;i<=x;++i) {
    printf("const rule& r"i);
    if (i==x)
      print ") {";
    else {
      printf(",");
      if (i%4==0)
	print "";
    }
  }
  for (i=1;i<=x;++i) {
    printf("rules.push_back(r"i".clone()); ");
    if (i==x) {
      print "";
      print "}";
    } else if (i%2==0) {
	print "";
    }
  }
}

	 
#const rule& r1,const rule& r2) { 
#  for (i=1;i<=x;++i)

BEGIN {
  n=ARGV[ARGC-1];
  ARGV[ARGC]=""
  for (i=1;i<=n;++i)
    doit(i);
}
