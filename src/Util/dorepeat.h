#ifndef dorepeat

//dorepeat(n) foo
//repeats foo n times
//maybe its mean to other people use this, but its certainly nicer that typing 
//for (int i=0;i<n;++i) when you never use i!

#define __DOREPEAT_CONCAT_3_( a, b ) a##b
#define __DOREPEAT_CONCAT_2_( a, b ) __DOREPEAT_CONCAT_3_( a, b )
#define __DOREPEAT_CONCAT( a, b ) __DOREPEAT_CONCAT_2_( a, b )
#define __DOREPEAT_UNIQUE_NAME __DOREPEAT_CONCAT( DOREPEAT_UNIQUE_NAME_, __LINE__ )

#define dorepeat(N) \
for (unsigned int __DOREPEAT_UNIQUE_NAME=N; __DOREPEAT_UNIQUE_NAME>0;--__DOREPEAT_UNIQUE_NAME)

#endif
