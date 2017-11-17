# redhawk-hackrf
Drivers for interfacing HackRF radios with REDHAWK

# Contents
* REDHAWK Softpkg Dependency (captivation.libhackrf)
* REDHAWK Device (captivation.HackRFOne)
* REDHAWK Node (captivation.HackRFNode)

# Instructions
To install into standard SDRROOT directory:
```bash
make install
```
To clean project:
```bash
make distclean
```

To run the HackRF device:
```bash
# Launch REDHAWK domain (if necessary)
nodeBooter -D&

# Launch node containing HackRF device
sudo make check
```
