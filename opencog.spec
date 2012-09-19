#
# spec file for RPM package opencog
#
# Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
# All Rights Reserved
#
# This file and all modifications and additions to the pristine
# package are under the same license as the package itself.
#
# Please submit bugfixes or comments to gama@vettalabs.com 
#

Name:           opencog
Version:        0.1.4
Release:        0
Summary:        A platform to build and share artificial intelligence programs

Group:          Development/Libraries
License:        AGPLv3
URL:            http://opencog.org
Source0:        file:///home/gama/rpmbuild/SOURCES/opencog-%{version}.tar.bz2
BuildRoot:      %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)


BuildRequires:  cmake >= 2.6, make >= 3, gcc-c++ >= 4.4, boost-devel >= 1.44, unixODBC-devel >= 2.2, guile-devel >= 1.8.6, gsl-devel >= 1.9, glibc-devel >= 2.3
%if 0%{?suse_version} > 1020
BuildRequires:  libexpat-devel >= 2
%else
BuildRequires:  expat-devel >= 2
%endif

%description
The Open Cognition Framework (OpenCog) provides research scientists and
software developers with a common platform to build and share artificial
intelligence programs. The OpenCog Framework framework includes:
    * a flexible and highly optimized in-memory database for knowledge
      representation,
    * a plug-in architecture for cognitive algorithms and a cognitive process
      scheduler,
    * a built-in LISP-like programming language, and
    * other components to support artificial intelligence research and
      development.

%prep
%setup -q


%build
cmake -DCMAKE_INSTALL_PREFIX:STRING=/usr -DCONFDIR=/etc -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo .
make %{?_smp_mflags}


# disable fedora's automatic debug package generation
%define debug_package %{nil}

%install
rm -rf %buildroot
mkdir %buildroot
make install DESTDIR=%buildroot


%package -n libatomspace
Summary: The underlying hyper-graph infrastructure of the Open Cognition Framework
Group: Development/Libraries
AutoReqProv: 0
Requires: libstdc++ >= 4.4, glibc >= 2.3
%description -n libatomspace
The libatomspace library provides the underlying hyper-graph (Nodes, Links,
TruthValues, AttentionValues, AtomTable, etc.) infrastructure of the Open
Cognition Framework.
%files -n libatomspace
%defattr(-,root,root,-)
%doc LICENSE
/%{_libdir}/opencog/libatomspace.so


%package -n libatomspace-devel
Summary: Header files for the libatomspace library
Group: Development/Libraries
AutoReqProv: 0
Requires: libatomspace >= %{version}, gcc-c++ >= 4.4, boost-devel >= 1.44, glibc-devel >= 2.3
%description -n libatomspace-devel
The libatomspace-devel package contains the header files needed to develop
programs that use the libatomspace library (which is part of the Open Cognition
Framework).
%files -n libatomspace-devel
%defattr(-,root,root,-)
%doc LICENSE
/%{_includedir}/opencog/atomspace/*
/%{_includedir}/opencog/util/*


%package server
Summary: The base server of the Open Cognition Framework
Group: Development/Libraries
AutoReqProv: 0
Requires: libatomspace >= %{version}, boost >= 1.44
%if 0%{?suse_version} > 1020
Requires: libexpat1 >= 2
%else
Requires: expat >= 2
%endif
%description server
The opencog-server package contains the base server and essential modules used
by the Open Cognition Framework.
%files server
%defattr(-,root,root,-)
%doc LICENSE
/%{_bindir}/cogserver
/%{_libdir}/opencog/libbuiltinreqs.so
/%{_libdir}/opencog/libserver.so
/%{_libdir}/opencog/libxml.so
%config(noreplace) /%{_sysconfdir}/opencog.conf


%package server-devel
Summary: Header files and development files for the opencog-server
Group:   Development/Libraries
AutoReqProv: 0
Requires: opencog-server >= %{version}, libatomspace-devel >= %{version}
%if 0%{?suse_version} > 1020
Requires: libexpat-devel >= 2
%else
Requires: expat-devel >= 2
%endif
%description server-devel
The opencog-server-devel package contains the header files and cmake scripts
needed to develop programs that use the Open Cognition Framework.
%files server-devel
%defattr(-,root,root,-)
%doc LICENSE README
/%{_includedir}/opencog/server/*
/%{_includedir}/opencog/persist/*
/%{_datadir}/opencog/cmake


%package core
Summary: The main modules used by the opencog server
Group:   Development/Libraries
AutoReqProv: 0
Requires: opencog-server >= %{version}, guile >= 1.8.6, unixODBC >= 2.2, gsl >= 1.9
%description core
The opencog-core package contains the modules shipped by default by the Open
Cognition Framework. They provide a variety of enhancements and additional
functionality such as scheme bindings, SQL-based persistence and attention
allocation.
%files core
%defattr(-,root,root,-)
%doc LICENSE
/%{_libdir}/opencog/libattention.so
/%{_libdir}/opencog/libdotty.so
/%{_libdir}/opencog/libpersist.so
/%{_libdir}/opencog/libquery.so
/%{_libdir}/opencog/libscheme.so
/%{_libdir}/opencog/libwsd.so
/%{_datadir}/opencog/scm/*


%package core-devel
Summary: Header files for the the opencog-core package
Group:   Development/Libraries
AutoReqProv: 0
Requires: opencog-core >= %{version}, opencog-server-devel >= %{version}, guile-devel >= 1.8.6, unixODBC-devel >= 2.2, gsl-devel >= 1.9
%description core-devel
The opencog-core-devel package contains the header files associated with the
modules that are part of the opencog-core package.
%files core-devel
%defattr(-,root,root,-)
%doc LICENSE
/%{_includedir}/opencog/dynamics/*
/%{_includedir}/opencog/guile/*
/%{_includedir}/opencog/nlp/*
/%{_includedir}/opencog/persist/*
/%{_includedir}/opencog/query/*

# add explicit debug packages as we had to disable debug_package as it is
# automatic included in fedora and thus causes a conflict when we try to
# include it to build in other distros
%package debuginfo
%global __debug_package 1
Summary: Debug information for package %{name}
Group: Development/Debug
AutoReqProv: 0
Requires: %{?!debug_package_requires:%{name} = %{version}-%{release}}%{?debug_package_requires}
%description debuginfo
This package provides debug information for package %{name}.
Debug information is useful when developing applications that use this
package or when debugging this package.
%files debuginfo -f debugfiles.list
%defattr(-,root,root)


%if 0%{?suse_version}
%package debugsource
Summary: Debug sources for package %{name}
Group: Development/Debug
AutoReqProv: 0
Requires: %{name}-debuginfo = %{version}-%{release}
%description debugsource
This package provides debug sources for package %{name}.
Debug sources are useful when developing applications that use this
package or when debugging this package.
%files debugsource -f debugsources.list
%defattr(-,root,root)
%endif


%clean
rm -rf %buildroot


%changelog
* Wed Oct 01 2008 Gustavo Machado Campagnani Gama <gama@vettalabs.com> 0.1.4-0
- Initial RPM release
* Wed Sep 10 2012 Linas Vepstas <linasvepstas@gmail.com>
- Remove obviously obsolete packages, dependencies.
