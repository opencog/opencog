/**
 * MessagingSystemExceptions.cc
 *
 *
 * Copyright(c) 2001 Thiago Maia, Andre Senna
 * All rights reserved.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "MessagingSystemExceptions.h"

namespace MessagingSystem {

CantBindToPortException::CantBindToPortException(int port) {
    fprintf(stderr, "Can't bind to port %d\n", port);
    fflush(stdout);
}

BrokedPortBindingException::BrokedPortBindingException(int port) {
    fprintf(stderr, "Binding to port %d is broken\n", port);
    fflush(stdout);
}

}
