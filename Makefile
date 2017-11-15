.PHONY: all install clean distclean

all:
	(cd sdr/libraries/captivation.libhackrf/cpp && ./reconf; ./configure; make -j2)
	(cd sdr/devices/captivation.HackRFOne/cpp && ./reconf; ./configure; make -j2)

install:
	(cd sdr/libraries/captivation.libhackrf/cpp && ./reconf; ./configure; make install -j2)
	(cd sdr/devices/captivation.HackRFOne/cpp && ./reconf; ./configure; make install -j2)
	(cp -r sdr/nodes/* /var/redhawk/sdr/dev/nodes/)

clean:
	(cd sdr/libraries/captivation.libhackrf/cpp && make clean)
	(cd sdr/devices/captivation.HackRFOne/cpp && make clean)

distclean:
	(cd sdr/libraries/captivation.libhackrf/cpp && make distclean)
	(cd sdr/devices/captivation.HackRFOne/cpp && make distclean)
