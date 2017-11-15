#!/bin/bash
nodeBooter=/usr/local/redhawk/core/bin/nodeBooter
node=/var/redhawk/sdr/dev/nodes/captivation/HackRFNode/DeviceManager.dcd.xml
libhackrf=/var/redhawk/sdr/dom/deps/captivation/libhackrf/cpp/lib

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$libhackrf
$nodeBooter -d $node
