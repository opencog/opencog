# Copyright 2008-2008 OpenCog.org
# Distributed under the terms of the GNU General Public License v2

inherit eutils

DESCRIPTION="A JUnit/CppUnit/xUnit-like framework for C++."
HOMEPAGE="http://cxxtest.sourceforge.net/"
SRC_URI="mirror://sourceforge/${PN}/${P}.tar.gz"
LICENSE="LGPL"
SLOT="0"
KEYWORDS="~alpha ~amd64 ~hppa ~ia64 ~ppc ~ppc64 ~sparc ~x86"
IUSE="python perl doc"

RDEPEND="python? (>=dev-lang/python-2.3)
		 perl? (>=dev-lang/perl-5)"

S=${WORKDIR}/${PN}

src_install() {
	if use doc; then
		insinto "/usr/share/doc/${P}"
	   	doins -r docs sample
	fi
	dodoc README TODO

	use perl   && dobin cxxtestgen.pl
	use python && dobin cxxtestgen.py

	insinto /usr/include/cxxtest
	doins cxxtest/*
}
