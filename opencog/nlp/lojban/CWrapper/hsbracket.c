#include <HsFFI.h>

static void my_enter(void) __attribute__((constructor));
static void my_enter(void)
{
  static char *argv[] = { "libopencog-lojban.so", 0 }, **argv_ = argv;
  static int argc = 1;
  hs_init(&argc, &argv_);
}

static void my_exit(void) __attribute__((destructor));
static void my_exit(void)
{
  hs_exit();
}
