INTRODUCTION
------------
This package contains the source for Ocaml interface to LinkGrammar.
It enables Ocaml applications to use LinkGrammar to parse sentences.

The Ocaml API maps closely to LinkGrammar's C API.

This package interfaces to the LinkGrammar C-API using the OCaml-C
communication. 

WARNING
-------
This library is untested and unsupported at this time.


PACKAGE
-------
Files are:
  - README.txt this file
  - the library:
      . linkgrammar.mli      Interface to LinkGrammar
      . linkgrammar.ml       Implementation
      . linkgrammar.c        OCaml API to LinkGrammar C API Interfacing
  - API application:
      . utBatch.ml       API unit tests. Parses sentences from file
      . utApi.ml         Unit Tests of API functions


INSTALLATION
------------
4. build the C file
gcc -I../link-4.1b/include -I/c/Program\ Files/Objective\ Caml/lib -c linkgrammar.c
4. build the ocaml library
   ocamlc -c linkgrammar.mli
   ocamlc -custom linkgrammar.o ../link-4.1b/link.a -a -o linkgrammar.cma linkgrammar.ml
5. build the applications
   ocamlc -o utBatch linkgrammar.cma utBatch.ml
   ocamlc -o utApi linkgrammar.cma utApi.ml	


DOCUMENTATION
-------------

This OCaml API closely follows the LinkGrammar C API. Please refer
http://www.abisource.com/projects/link-grammar/ for details of the API usage


AUTHOR
------
Ramu Ramamurthy - ramu_ramamurthy at yahoo dot com

LICENSE (BSD)
-------------
Copyright (c) 2006, Ramu Ramamurthy
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of the author nor the names of its contributors
      may be used to endorse or promote products derived from this
      software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

