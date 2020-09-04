## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,e674 linker.cmd package/cfg/dss_mmw_pe674.oe674

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/dss_mmw_pe674.xdl
	$(SED) 's"^\"\(package/cfg/dss_mmw_pe674cfg.cmd\)\"$""\"D:/ti/mmwave_sdk_01_02_00_05/packages/ti/demo/xwr16xx/mmwCanTx/dss/mmw_configPkg_xwr16xx/\1\""' package/cfg/dss_mmw_pe674.xdl > $@
	-$(SETDATE) -r:max package/cfg/dss_mmw_pe674.h compiler.opt compiler.opt.defs
