%define name moses
%define version 3.2.1
%define release 1

Summary: MOSES automated program learning system

Name: %{name}
Version: %{version}
Release: %{release}
Group: System Environment/Base
License: Apache2

Source: %{name}-%{version}.tar.bz2
# Still need to specify BuildRoot, as otherwise RPM_BUILD_ROOT is not set, below.
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root
URL: http://opencog.org/wiki/MOSES

#Requires: 
Requires(post): /sbin/ldconfig
Requires(postun): /sbin/ldconfig

BuildRequires: cmake 

%package devel
Group:  Development/C
Summary: Headers and libraries needed to develop for MOSES
Requires: %{name} = %{version}

%description
MOSES, the Meta-Optimizing Semantic Evolutionary Search system, is a 
supervised machine learning system designed to automatically learn
programs that produce a specific output, given input data.

%description devel
Headers and libraries needed to develop for MOSES

%prep

%setup

%build

CFLAGS="$RPM_OPT_FLAGS" 
mkdir build
cd build
cmake ..

if [ "$SMP" != "" ]; then
  (%__make "MAKE=%__make -k -j $SMP"; exit 0)
  %__make
else
%__make
fi

%install
cd build
if [ -d $RPM_BUILD_ROOT ]; then rm -rf $RPM_BUILD_ROOT; fi
%__make DESTDIR=$RPM_BUILD_ROOT install

%files
%defattr(644,root,root,755)
%doc LICENSE README
%attr(755,root,root)%{_bindir}/*
%{_libdir}/lib*.so*

%clean
%__rm -rf $RPM_BUILD_ROOT

%post
/sbin/ldconfig

%postun
/sbin/ldconfig

%files devel
%defattr(-,root,root)
%{_libdir}/lib*.a
%{_libdir}/lib*.la


%changelog
* Fri Jun 15 2012 Linas Vepstas <linavepstas@gmail.com>
- Initial version

