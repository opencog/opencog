/*
 * Copyright (c) 2004-2009 Sergey Lyubka
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * $Id: mongoose.h 471 2009-08-30 14:30:21Z valenok $
 */

#ifndef MONGOOSE_HEADER_INCLUDED
#define	MONGOOSE_HEADER_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct mg_context;	/* Handle for the HTTP service itself	*/
struct mg_connection;	/* Handle for the individual connection	*/


/*
 * This structure contains full information about the HTTP request.
 * It is passed to the user-specified callback function as a parameter.
 */
struct mg_request_info {
	char	*request_method;	/* "GET", "POST", etc	*/
	char	*uri;			/* Normalized URI	*/
	char	*http_version;		/* E.g. "1.0", "1.1"	*/
	char	*query_string;		/* \0 - terminated	*/
	char	*post_data;		/* POST data buffer	*/
	char	*remote_user;		/* Authenticated user	*/
	long	remote_ip;		/* Client's IP address	*/
	int	remote_port;		/* Client's port	*/
	int	post_data_len;		/* POST buffer length	*/
	int	status_code;		/* HTTP status code	*/
	int	num_headers;		/* Number of headers	*/
	struct mg_header {
		char	*name;		/* HTTP header name	*/
		char	*value;		/* HTTP header value	*/
	} http_headers[64];		/* Maximum 64 headers	*/
};


/*
 * User-defined callback function prototype for URI handling, error handling,
 * or logging server messages.
 */
typedef void (*mg_callback_t)(struct mg_connection *,
		const struct mg_request_info *info, void *user_data);


/*
 * Start the web server.
 * This must be the first function called by the application.
 * It creates a serving thread, and returns a context structure that
 * can be used to alter the configuration, and stop the server.
 */
struct mg_context *mg_start(void);


/*
 * Stop the web server.
 * Must be called last, when an application wants to stop the web server and
 * release all associated resources. This function blocks until all Mongoose
 * threads are stopped. Context pointer becomes invalid.
 */
void mg_stop(struct mg_context *);


/*
 * Return current value of a particular option.
 */
const char *mg_get_option(const struct mg_context *, const char *option_name);


/*
 * Set a value for a particular option.
 * Mongoose makes an internal copy of the option value string, which must be
 * valid nul-terminated ASCII or UTF-8 string. It is safe to change any option
 * at any time. The order of setting various options is also irrelevant with
 * one exception: if "ports" option contains SSL listening ports, a "ssl_cert"
 * option must be set BEFORE the "ports" option.
 * Return value:
 *	-1 if option is unknown
 *	0  if mg_set_option() failed
 *	1  if mg_set_option() succeeded 
 */
int mg_set_option(struct mg_context *, const char *opt_name, const char *value);


/*
 * Add, edit or delete the entry in the passwords file.
 * This function allows an application to manipulate .htpasswd files on the
 * fly by adding, deleting and changing user records. This is one of the two
 * ways of implementing authentication on the server side. For another,
 * cookie-based way please refer to the examples/authentication.c in the
 * source tree.
 * If password is not NULL, entry is added (or modified if already exists).
 * If password is NULL, entry is deleted. Return:
 *	1 on success
 *	0 on error 
 */
int mg_modify_passwords_file(struct mg_context *ctx, const char *file_name,
		const char *user_name, const char *password);


/*
 * Register URI handler.
 * It is possible to handle many URIs if using * in the uri_regex, which
 * matches zero or more characters. user_data pointer will be passed to the
 * handler as a third parameter. If func is NULL, then the previously installed
 * handler for this uri_regex is removed.
 */
void mg_set_uri_callback(struct mg_context *ctx, const char *uri_regex,
		mg_callback_t func, void *user_data);


/*
 * Register HTTP error handler.
 * An application may use that function if it wants to customize the error
 * page that user gets on the browser (for example, 404 File Not Found message).
 * It is possible to specify a error handler for all errors by passing 0 as
 * error_code. That '0' error handler must be set last, if more specific error
 * handlers are also used. The actual error code value can be taken from
 * the request info structure that is passed to the callback.
 */
void mg_set_error_callback(struct mg_context *ctx, int error_code,
		mg_callback_t func, void *user_data);


/*
 * Register authorization handler.
 * This function provides a mechanism to implement custom authorization,
 * for example cookie based (look at examples/authorization.c).
 * The callback function must analyze the request, and make its own judgement
 * on wether it should be authorized or not. After the decision is made, a
 * callback must call mg_authorize() if the request is authorized.
 */
void mg_set_auth_callback(struct mg_context *ctx, const char *uri_regex,
		mg_callback_t func, void *user_data);


/*
 * Register log handler.
 * By default, Mongoose logs all error messages to stderr. If "error_log"
 * option is specified, the errors are written in the specified file. However,
 * if an application registers its own log handler, Mongoose will not log
 * anything but call the handler function, passing an error message as
 * "user_data" callback argument.
 */
void mg_set_log_callback(struct mg_context *ctx, mg_callback_t func);


/*
 * Register SSL password handler.
 * This is needed only if SSL certificate asks for a password. Instead of
 * prompting for a password on a console a specified function will be called.
 */
typedef int (*mg_spcb_t)(char *buf, int num, int w, void *key);
void mg_set_ssl_password_callback(struct mg_context *ctx, mg_spcb_t func);


/*
 * Send data to the browser.
 * Return number of bytes sent. If the number of bytes sent is less then
 * requested or equals to -1, network error occured, usually meaning the
 * remote side has closed the connection.
 */
int mg_write(struct mg_connection *, const void *buf, int len);


/*
 * Send data to the browser using printf() semantics.
 * Works exactly like mg_write(), but allows to do message formatting.
 * Note that mg_printf() uses internal buffer of size MAX_REQUEST_SIZE
 * (8 Kb by default) as temporary message storage for formatting. Do not
 * print data that is bigger than that, otherwise it will be truncated.
 * Return number of bytes sent.
 */
int mg_printf(struct mg_connection *, const char *fmt, ...);


/*
 * Read data from the remote or local end.
 */
int mg_read(struct mg_connection *, int local, void *buf, int len);

/*
 * Get the value of particular HTTP header.
 * This is a helper function. It traverses request_info->http_headers array,
 * and if the header is present in the array, returns its value. If it is
 * not present, NULL is returned.
 */
const char *mg_get_header(const struct mg_connection *, const char *hdr_name);


/*
 * Authorize the request.
 * See the documentation for mg_set_auth_callback() function.
 */
void mg_authorize(struct mg_connection *);


/*
 * Get a value of particular form variable.
 * Both query string (whatever comes after '?' in the URL) and a POST buffer
 * are scanned. If a variable is specified in both query string and POST
 * buffer, POST buffer wins. Return value:
 *	NULL      if the variable is not found
 *	non-NULL  if found. NOTE: this returned value is dynamically allocated
 *		  and is subject to mg_free() when no longer needed. It is
 *		  an application's responsibility to mg_free() the variable. 
 */
char *mg_get_var(const struct mg_connection *, const char *var_name);


/*
 * Free up memory returned by mg_get_var().
 */
void mg_free(char *var);


/*
 * Return Mongoose version.
 */
const char *mg_version(void);


/*
 * Print command line usage string.
 */
void mg_show_usage_string(FILE *fp);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MONGOOSE_HEADER_INCLUDED */
