
/**
 * sniff test for the memcached library
 *
 * Linas Vepstas June 2008
 */

#ifdef HAVE_LIBMEMCACHED

#include <assert.h>
#include <memcached.h>
#include <stdio.h>

int main (int argc, char *argv[])
{
	memcached_return rc;
	memcached_st *mc;
	mc = memcached_create(NULL);

	memcached_server_st *servers;
	// char servername[] = "localhost";
	char servername[] = "127.0.0.1";
	int port_number = 21201;
	servers = memcached_server_list_append(NULL, servername, port_number, &rc);

	rc = memcached_server_push(mc, servers);
	assert(rc == MEMCACHED_SUCCESS);
	memcached_server_list_free(servers);

#if 0
	rc = memcached_set (mc, "asdf", 4, "pqrs", 4, 0, 0);
	if (rc != MEMCACHED_SUCCESS)
	{
		printf("setting -- oops its %s\n", memcached_strerror(mc, rc));
	}
	else
	{
		printf("store success!\n");
	}
#endif

	size_t vlen;
	uint32_t flags;
	char *val = memcached_get(mc, "asdf", 4, &vlen, &flags, &rc);
	if (rc != MEMCACHED_SUCCESS)
	{
		printf("reading -- oops its %s\n", memcached_strerror(mc, rc));
	}
	printf ("read val=%s len=%d\n", val, vlen);
	

	memcached_free(mc);

	return 0;
}
#else /* HAVE_LIBMEMCACHED */
int main (int argc, char * argv[]) { return 0; }
#endif /* HAVE_LIBMEMCACHED */
