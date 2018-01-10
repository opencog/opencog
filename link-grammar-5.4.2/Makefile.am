#
# Master Makefile for the Link Grammar system
#
SUBDIRS =               \
   link-grammar         \
   viterbi              \
   bindings             \
   link-parser          \
   tests                \
   morphology           \
   data                 \
   man

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = link-grammar.pc

ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST =            \
	autogen.sh           \
	link-grammar.pc.in   \
	link-grammar.spec.in \
	link-grammar.spec    \
	AUTHORS              \
	ChangeLog            \
	FindLinkGrammar.cmake\
	LICENSE              \
	MAINTAINERS          \
	NEWS                 \
	README.md            \
	docker/docker-build.sh                 \
	docker/docker-base/Dockerfile          \
	docker/docker-parser/Dockerfile        \
	docker/docker-python/Dockerfile        \
	docker/docker-server/Dockerfile        \
	m4/varcheckpoint.m4
	msvc14/LGlib-features.props            \
	msvc14/LinkGrammarExe.vcxproj          \
	msvc14/LinkGrammarExe.vcxproj.filters  \
	msvc14/LinkGrammarJava.vcxproj         \
	msvc14/LinkGrammarJava.vcxproj.filters \
	msvc14/LinkGrammar.sln                 \
	msvc14/LinkGrammar.vcxproj             \
	msvc14/LinkGrammar.vcxproj.filters     \
	msvc14/Local.props                     \
	msvc14/confvar.bat                     \
	msvc14/MSVC-common.props               \
	msvc14/post-build.bat                  \
	msvc14/Python2.vcxproj                 \
	msvc14/Python2.vcxproj.filters         \
	msvc14/Python3.vcxproj.filters         \
	msvc14/README.md                       \
	msvc14/make-check.py                   \
	mingw/README.Cygwin                    \
	mingw/README.MSYS                      \
	TODO
