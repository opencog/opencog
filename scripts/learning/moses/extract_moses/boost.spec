%define underb 1_49_0


Summary: The Boost C++ Libraries
Name: boost
Version: 1.49.0
Release: 1
Source: %{name}_%{underb}.tar.bz2
License: Boost Software License
Group: System Environment/Libraries
# BuildRoot: %{_builddir}/%{name}-%{version}-root
BuildRoot: %{_tmppath}/%{name}-%{version}-root
URL: http://www.boost.org/
Prereq: /sbin/ldconfig
BuildRequires: libstdc++-devel python python-devel bzip2-devel 
Obsoletes: boost < 1.49
Obsoletes: boost-devel < 1.49
Obsoletes: boost-doc < 1.49

#Requires: 
Requires(post): /sbin/ldconfig
Requires(postun): /sbin/ldconfig


%description
Boost provides free peer-reviewed portable C++ source libraries. The
emphasis is on libraries which work well with the C++ Standard
Library. One goal is to establish "existing practice" and provide
reference implementations so that the Boost libraries are suitable for
eventual standardization. (Some of the libraries have already been
proposed for inclusion in the C++ Standards Committee's upcoming C++
Standard Library Technical Report.)

%package devel
Summary: The Boost C++ Headers
Group: System Environment/Libraries
Requires: boost = %{version}-%{release}

%description devel
Headers for the Boost C++ libraries

%prep

%setup -q -n %{name}_%{underb}

%build
./bootstrap.sh
./b2 --prefix=$RPM_BUILD_ROOT

%install
rm -rf $RPM_BUILD_ROOT
mkdir -p $RPM_BUILD_ROOT/usr
./b2 --prefix=$RPM_BUILD_ROOT/usr install
mv $RPM_BUILD_ROOT/usr/lib $RPM_BUILD_ROOT/usr/lib64

%clean
rm -rf $RPM_BUILD_ROOT

%files
%defattr(-,root,root)
%{_libdir}/*.so.%{version}
%{_libdir}/*.so

%files devel
%defattr(-, root, root)
%{_includedir}/boost
%{_libdir}/*.a

%changelog
* Fri Sep 14 2012 Linas Vepstas <linavepstas@gmail.com>
- Hacked for 1.49

* Thu Feb 12 2009 Tanmay
- Initial version
  http://sidekick.windforwings.com/2009/02/rpm-spec-for-boost-libboost-rpm-on.html
