
/*
 * SchemeShell.c
 *
 * Simple scheme shell
 */

class SchemeShell
{
	private:
		static bool is_inited = false;
		void register_procs(void);

	public:
		SchemeShell(void);
};


#include <guile/gh.h>
#include <libguile.h>
#include <libguile/backtrace.h>


SchemeShell:SchemeShell(void)
{
	if (!is_inited)
	{
		is_inited = true;
		scm_init_debug();
		scm_init_backtrace();
		register_procs();
	}
}

static SCM ss_hello (void)
{
	printf("hello world\n");
	return SCM_EOL;
}

void SchemeShell:register_procs(void)
{
	scm_c_define_gsubr("cog-hello",               0, 0, 0, ss_hello);
}
