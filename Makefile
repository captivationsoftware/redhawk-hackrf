.PHONY: all install clean distclean check

all:
	(cd sdr/libraries/captivation.libhackrf/cpp && ./reconf; ./configure; make -j2)
	(cd sdr/devices/captivation.HackRFOne/cpp && ./reconf; ./configure; make -j2)

install:
	(cd sdr/libraries/captivation.libhackrf/cpp && ./reconf; ./configure; make install -j2)
	(cd sdr/devices/captivation.HackRFOne/cpp && ./reconf; ./configure; make install -j2)
	mkdir -p /var/redhawk/sdr/dev/nodes/captivation/
	cp -r sdr/nodes/captivation.HackRFNode /var/redhawk/sdr/dev/nodes/captivation/HackRFNode

clean:
	(cd sdr/libraries/captivation.libhackrf/cpp && make clean)
	(cd sdr/devices/captivation.HackRFOne/cpp && make clean)

distclean:
	(cd sdr/libraries/captivation.libhackrf/cpp && make distclean)
	(cd sdr/devices/captivation.HackRFOne/cpp && make distclean)

check:
	@echo "Warn: 'sudo' permissions may be necessary for device to access USB"
	./scripts/launch_hackrf.sh
