# Copyright 2008-2008 OpenCog.org
# Distributed under the terms of the GNU General Public License v2

inherit eutils

DESCRIPTION="C++ Sockets is a C++ wrapper for BSD-style sockets."
HOMEPAGE="http://www.alhem.net/Sockets/index.html"
SRC_URI="http://www.alhem.net/Sockets/Sockets-${PV}.tar.gz"
LICENSE="GPL-2"
SLOT="0"
KEYWORDS="~x86"
IUSE=""

DEPEND="sys-libs/glibc"
S="${WORKDIR}/Sockets-${PV}"

src_compile() {
	emake || die "emake failed"
}

src_install() {
	emake PREFIX="${D}/usr" install || die "emake install failed"
}
