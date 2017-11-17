#!/bin/bash
nodeBooter=/usr/local/redhawk/core/bin/nodeBooter
node=/var/redhawk/sdr/dev/nodes/captivation/HackRFNode/DeviceManager.dcd.xml

$nodeBooter -d $node
