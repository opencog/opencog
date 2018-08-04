
/* 
 * Socket I/O to the opencog reasoning engine
 * Copied from "whirr.c". 
 * Linas October 2007 
 */

#ifdef __cplusplus
extern "C" {
#endif

void whirr_sock_setup (void);
char * whirr_sock_io (const char * msg);

#ifdef __cplusplus
}
#endif

