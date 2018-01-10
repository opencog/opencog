/*************************************************************************/
/* Copyright (c) 2004                                                    */
/* Daniel Sleator, David Temperley, and John Lafferty                    */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the link grammar parsing system is subject to the terms of the */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <time.h>

#include "externs.h"

#if !defined(_WIN32)
	#include <sys/time.h>
	#include <sys/resource.h>
#endif

#if defined(__linux__)
	/* based on reading the man page for getrusage on linux, I inferred that
	   I needed to include this.  However it doesn't seem to be necessary */
	#include <unistd.h>
#endif

#if defined(__hpux__)
	#include <sys/syscall.h>
	int syscall(int, int, struct rusage *rusage);  /* can't find
	                                                  the prototype for this */
	#define getrusage(a, b)  syscall(SYS_GETRUSAGE, (a), (b))
#endif /* __hpux__ */

#if defined(__sun__)
int getrusage(int who, struct rusage *rusage);
/* Declaration missing from sys/resource.h in sun operating systems (?) */
#endif /* __sun__ */

#include "api-structures.h"
#include "resources.h"
#include "utilities.h"

#define MAX_PARSE_TIME_UNLIMITED -1
#define MAX_MEMORY_UNLIMITED ((size_t) -1)

/** returns the current usage time clock in seconds */
static double current_usage_time(void)
{
#if !defined(_WIN32)
	struct rusage u;
	getrusage (RUSAGE_SELF, &u);
	return (u.ru_utime.tv_sec + ((double) u.ru_utime.tv_usec) / 1000000.0);
#else
	return ((double) clock())/CLOCKS_PER_SEC;
#endif
}

Resources resources_create(void)
{
	Resources r;
	double now;

	r = (Resources) xalloc(sizeof(struct Resources_s));
	r->max_parse_time = MAX_PARSE_TIME_UNLIMITED;
	now = current_usage_time();
	r->when_created = now;
	r->when_last_called = now;
	r->time_when_parse_started = now;
	r->space_when_parse_started = get_space_in_use();
	r->max_memory = MAX_MEMORY_UNLIMITED;
	r->cumulative_time = 0;
	r->memory_exhausted = false;
	r->timer_expired = false;

	return r;
}

void resources_delete(Resources r)
{
	xfree(r, sizeof(struct Resources_s));
}

void resources_reset(Resources r)
{
	r->when_last_called = r->time_when_parse_started = current_usage_time();
	r->space_when_parse_started = get_space_in_use();
	r->timer_expired = false;
	r->memory_exhausted = false;
}

#if 0
static void resources_reset_time(Resources r)
{
	r->when_last_called = r->time_when_parse_started = current_usage_time();
}
#endif

void resources_reset_space(Resources r)
{
	r->space_when_parse_started = get_space_in_use();
}

bool resources_exhausted(Resources r)
{
	if (r->timer_expired || r->memory_exhausted)
		return true;

	if (resources_timer_expired(r))
		r->timer_expired = true;

	if (resources_memory_exhausted(r))
		r->memory_exhausted = true;

	return (r->timer_expired || r->memory_exhausted);
}

bool resources_timer_expired(Resources r)
{
	if (r->max_parse_time == MAX_PARSE_TIME_UNLIMITED) return false;
	else return (r->timer_expired ||
	     (current_usage_time() - r->time_when_parse_started > r->max_parse_time));
}

bool resources_memory_exhausted(Resources r)
{
	if (r->max_memory == MAX_MEMORY_UNLIMITED) return false;
	else return (r->memory_exhausted || (get_space_in_use() > r->max_memory));
}

#define RES_COL_WIDTH sizeof("                                     ")

/** print out the cpu ticks since this was last called */
static void resources_print_time(int verbosity_opt, Resources r, const char * s)
{
	double now;
	now = current_usage_time();
	if (verbosity_opt >= D_USER_TIMES)
	{
		prt_error("++++ %-36s %7.2f seconds\n", s, now - r->when_last_called);
	}
	r->when_last_called = now;
}

/** print out the cpu ticks since this was last called */
static void resources_print_total_time(int verbosity_opt, Resources r)
{
	double now;
	now = current_usage_time();
	r->cumulative_time += (now - r->time_when_parse_started) ;
	if (verbosity_opt >= D_USER_BASIC)
	{
		prt_error("++++ %-36s %7.2f seconds (%.2f total)\n", "Time",
		          now - r->time_when_parse_started, r->cumulative_time);
	}
	r->time_when_parse_started = now;
}

static void resources_print_total_space(int verbosity_opt, Resources r)
{
	if (verbosity_opt >= D_USER_TIMES)
	{
		prt_error("++++ %-36s %zu bytes (%zu max)\n", "Total space",
		          get_space_in_use(), get_max_space_used());
	}
}

void print_time(Parse_Options opts, const char * s)
{
	resources_print_time(opts->verbosity, opts->resources, s);
}

void parse_options_print_total_time(Parse_Options opts)
{
	resources_print_total_time(opts->verbosity, opts->resources);
}

void print_total_space(Parse_Options opts)
{
	resources_print_total_space(opts->verbosity, opts->resources);
}

