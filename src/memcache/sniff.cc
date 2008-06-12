
/**
 * sniff test for the memcached library
 *
 * Linas Vepstas June 2008
 */

#include <memcached.h>

int main ()
{
	memcached_return rc;
	memcached_st *mc;
	mc = memcached_create(NULL);

	memcached_server_st *servers;
	char servername[] = "localhost";
	servers = memcached_server_list_append(NULL, servername, 400, &rc);

	rc = memcached_server_push(mc, servers);
	memcached_server_list_free(servers);



	memcached_set (mc, "asdf", 4, "pqrs", 4, 0, 0);

	memcached_free(mc);
}
