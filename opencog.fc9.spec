Name:           opencog
Version:        0.1.3
Release:        1.fc9
Summary:        The OpenCog framework provides research scientists and software developers with a common platform to build and share artificial intelligence programs.

Group:          Development/Libraries
License:        AGPLv3
URL:            http://opencog.org
Source0:        file:///home/gama/rpmbuild/SOURCES/opencog-%{version}.tar.gz
BuildRoot:      %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)

BuildRequires:  cmake >= 2.4.6, make >= 3, gcc-c++ >= 4.1, csockets-devel >= 2.2.9, expat-devel >= 2, boost-devel >= 1.34, unixODBC-devel >= 2.2, guile-devel >= 1.8.5, glibc-devel >= 2.3

%description
The Open Cognition Framework (OpenCog) provides research scientists and software developers with a common platform to build and share artificial intelligence programs. The OpenCog Framework framework includes:
    * a flexible and highly optimized in-memory database for knowledge representation,
    * a plug-in architecture for cognitive algorithms and a cognitive process scheduler,
    * a built-in LISP-like programming language, and
    * other components to support artificial intelligence research and development.


%prep
%setup -q


%build
cmake -DCMAKE_INSTALL_PREFIX:STRING=/usr -DCONFDIR=/etc -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo .
make %{?_smp_mflags}


%install
rm -rf $RPM_BUILD_ROOT
make install DESTDIR=$RPM_BUILD_ROOT


%clean
rm -rf $RPM_BUILD_ROOT

%package -n libatomspace
Summary: The underlying hyper-graph infrastructure of the Open Cognition Framework.
Group: Development/Libraries
Requires: libstdc++ >= 4.1, glibc >= 2.3
%description -n libatomspace
The libatomspace library provides the underlying hyper-graph (Nodes,
Links, TruthValues, AttentionValues, AtomTable, etc.) infrastructure of
the Open Cognition Framework.
%files -n libatomspace
%defattr(-,root,root,-)
%doc
/%{_libdir}/opencog/libatomspace.so

%package -n libatomspace-devel
Summary: Header files for the libatomspace library.
Group: Development/Libraries
Requires: libatomspace >= %{version}, gcc-c++ >= 4.1, boost-devel >= 1.34, glibc-devel >= 2.3
%description -n libatomspace-devel
The libatomspace-devel package contains the header files needed to
develop programs that use the libatomspace library (which is part of the
Open Cognition Framework).
%files -n libatomspace-devel
%defattr(-,root,root,-)
%doc
/%{_includedir}/opencog/atomspace/*
/%{_includedir}/opencog/util/*

%package server
Summary: The base server of the Open Cognition Framework.
Group: Development/Libraries
Requires: libatomspace >= %{version}, openssl >= 0.9.8, expat >= 2, boost >= 1.34
%description server
The opencog-server package contains the base server and essential
modules used by the Open Cognition Framework.
%files server
%defattr(-,root,root,-)
%doc
/%{_bindir}/cogserver
/%{_libdir}/opencog/libbuiltinreqs.so
/%{_libdir}/opencog/libserver.so
%config(noreplace) /%{_sysconfdir}/opencog.conf

%package server-devel
Summary: Header files and development files for the opencog-server.
Group:   Development/Libraries
Requires: opencog-server >= %{version}, libatomspace-devel >= %{version}, csockets-devel >= 2.2.9, openssl-devel >= 0.9.8, expat-devel >= 2
%description server-devel
The opencog-server-devel package contains the header files and cmake
scripts needed to develop programs that use the Open Cognition
Framework.
%files server-devel
%defattr(-,root,root,-)
%doc
/%{_includedir}/opencog/server/*
/%{_includedir}/opencog/xml/*
/%{_datadir}/opencog/cmake

%package core
Summary: The main modules used by the opencog server.
Group:   Development/Libraries
%description core
The opencog-core package contains the modules shipped by default by the
Open Cognition Framework. They provide a variety of enhancements and
additional functionality such as scheme bindings, SQL-based persistence
and attention allocation.
%files core
%defattr(-,root,root,-)
%doc
/%{_libdir}/opencog/libattention.so
/%{_libdir}/opencog/libdotty.so
/%{_libdir}/opencog/libpersist.so
/%{_libdir}/opencog/libquery.so
/%{_libdir}/opencog/libscheme.so
/%{_libdir}/opencog/libwsd.so
/%{_datadir}/opencog/scm/*

%package core-devel
Summary: Header files for the the opencog-core package.
Group:   Development/Libraries
Requires: opencog-core >= %{version}, opencog-server-devel >= %{version}, guile-devel >= 1.8.5, unixODBC-devel >= 2.2
%description core-devel
The opencog-core-devel package contains the header files associated with
the modules that are part of the opencog-core package.
%files core-devel
%defattr(-,root,root,-)
%doc
/%{_includedir}/opencog/dynamics/*
/%{_includedir}/opencog/guile/*
/%{_includedir}/opencog/nlp/*
/%{_includedir}/opencog/persist/*
/%{_includedir}/opencog/query/*

%changelog
* Wed Sep 25 2008 Gustavo Machado Campagnani Gama <gama@vettalabs.com> 0.1.3-0.gama.1
- Initial RPM release
