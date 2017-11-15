# By default, the RPM will install to the standard REDHAWK SDR root location (/var/redhawk/sdr)
# You can override this at install time using --prefix /new/sdr/root when invoking rpm (preferred method, if you must)
%{!?_sdrroot: %global _sdrroot /var/redhawk/sdr}
%define _prefix %{_sdrroot}
Prefix:         %{_prefix}

# Point install paths to locations within our target SDR root
%define _sysconfdir    %{_prefix}/etc
%define _localstatedir %{_prefix}/var
%define _mandir        %{_prefix}/man
%define _infodir       %{_prefix}/info

Name:           captivation.HackRFOne
Version:        1.0.0
Release:        1%{?dist}
Summary:        Device %{name}

Group:          REDHAWK/Devices
License:        None
Source0:        %{name}-%{version}.tar.gz
BuildRoot:      %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)

BuildRequires:  redhawk-devel >= 2.1
Requires:       redhawk >= 2.1

BuildRequires:  captivation.libhackrf-devel
Requires:       captivation.libhackrf

# Interface requirements
BuildRequires:  frontendInterfaces >= 2.4 bulkioInterfaces >= 2.1
Requires:       frontendInterfaces >= 2.4 bulkioInterfaces >= 2.1


%description
Device %{name}
 * Commit: __REVISION__
 * Source Date/Time: __DATETIME__


%prep
%setup -q


%build
# Implementation cpp
pushd cpp
./reconf
%define _bindir %{_prefix}/dev/devices/captivation/HackRFOne/cpp
%configure
make %{?_smp_mflags}
popd


%install
rm -rf $RPM_BUILD_ROOT
# Implementation cpp
pushd cpp
%define _bindir %{_prefix}/dev/devices/captivation/HackRFOne/cpp
make install DESTDIR=$RPM_BUILD_ROOT
popd


%clean
rm -rf $RPM_BUILD_ROOT


%files
%defattr(-,redhawk,redhawk,-)
%dir %{_sdrroot}/dev/devices/captivation
%dir %{_sdrroot}/dev/devices/captivation/HackRFOne
%{_prefix}/dev/devices/captivation/HackRFOne/HackRFOne.scd.xml
%{_prefix}/dev/devices/captivation/HackRFOne/HackRFOne.prf.xml
%{_prefix}/dev/devices/captivation/HackRFOne/HackRFOne.spd.xml
%{_prefix}/dev/devices/captivation/HackRFOne/cpp

