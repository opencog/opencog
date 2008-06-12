
/**
 * sniff test for the memcached library
 *
 * Linas Vepstas June 2008
 */

#include <assert.h>
#include <memcached.h>
#include <stdio.h>

int main ()
{
	memcached_return rc;
	memcached_st *mc;
	mc = memcached_create(NULL);

	memcached_server_st *servers;
	// char servername[] = "localhost";
	char servername[] = "127.0.0.1";
	servers = memcached_server_list_append(NULL, servername, 21201, &rc);

	rc = memcached_server_push(mc, servers);
	assert(rc == MEMCACHED_SUCCESS);
	memcached_server_list_free(servers);

	
	rc = memcached_set (mc, "asdf", 4, "pqrs", 4, 0, 0);
	if (rc != MEMCACHED_SUCCESS)
	{
		printf("oops its %s\n", memcached_strerror(mc, rc));
	}

	size_t vlen;
	uint32_t flags;
	char *val = memcached_get(mc, "asdf", 4, &vlen, &flags, &rc);
	assert(rc == MEMCACHED_SUCCESS);
	printf ("duude val=%s len=%d\n", val, vlen);
	

	memcached_free(mc);
}
