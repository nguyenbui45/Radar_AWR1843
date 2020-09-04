# invoke SourceDir generated makefile for mss_mmw.per4ft
mss_mmw.per4ft: .libraries,mss_mmw.per4ft
.libraries,mss_mmw.per4ft: package/cfg/mss_mmw_per4ft.xdl
	$(MAKE) -f package/cfg/mss_mmw_per4ft.src/makefile.libs

clean::
	$(MAKE) -f package/cfg/mss_mmw_per4ft.src/makefile.libs clean

