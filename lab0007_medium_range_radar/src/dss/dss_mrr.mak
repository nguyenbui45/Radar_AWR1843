###################################################################################
# DSS MRR TI Design
#
#  NOTE:
#      (C) Copyright 2017 Texas Instruments, Inc.
###################################################################################
.PHONY: dssMRR dssMRRClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c dss
vpath %.c common
vpath %.c $(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft16x16/c64P
vpath %.c $(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P
vpath %.c ../../utils

###################################################################################
# Additional libraries which are required to build the design:
###################################################################################
DSS_MRR_TI_DESIGN_STD_LIBS = $(C674_COMMON_STD_LIB)	      			\
   			-llibedma_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)			\
   			-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)		\
   			-llibadcbuf_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)      	\
            -llibmmwavealg_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT) 	\
            -ldsplib.ae64P							\
            -lmathlib.$(C674_LIB_EXT)
DSS_MRR_TI_DESIGN_LOC_LIBS = $(C674_COMMON_LOC_LIB)					\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/edma/lib			\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib	        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/adcbuf/lib	        \
			-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/lib 		\
           	-i$(C674x_MATHLIB_INSTALL_PATH)/packages/ti/mathlib/lib		\
           	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/alg/mmwavelib/lib

###################################################################################
# DSS MRR TI Design
###################################################################################
DSS_MRR_CFG_PREFIX     = dss_mrr
DSS_MRR_TI_DESIGN_CFG       = dss/$(DSS_MRR_CFG_PREFIX).cfg
DSS_MRR_TI_DESIGN_ROV_XS    = $(DSS_MRR_CFG_PREFIX)_$(C674_XS_SUFFIX).rov.xs
DSS_MRR_TI_DESIGN_CONFIGPKG = dss/mrr_configPkg_$(MMWAVE_SDK_DEVICE_TYPE)
DSS_MRR_TI_DESIGN_MAP       = $(MMWAVE_SDK_DEVICE_TYPE)_mrr_ti_design_dss.map
DSS_MRR_DEMO_OUT       = $(MMWAVE_SDK_DEVICE_TYPE)_mrr_ti_design_dss.$(C674_EXE_EXT)
DSS_MRR_TI_DESIGN_BIN       = $(MMWAVE_SDK_DEVICE_TYPE)_mrr_ti_design_dss.bin
DSS_MRR_TI_DESIGN_CMD       = dss/dss_mrr_linker.cmd
DSS_MRR_TI_DESIGN_SOURCES   = dss_main.c \
                                dss_config_edma_util.c \
                                dss_data_path.c \
                                gen_twiddle_fft16x16.c \
                                gen_twiddle_fft32x32.c \
                                clusteringDBscan.c \
                                Extended_Kalman_Filter_xyz.c

DSS_MRR_TI_DESIGN_DEPENDS   = $(addprefix $(PLATFORM_OBJDIR)/, $(DSS_MRR_TI_DESIGN_SOURCES:.c=.$(C674_DEP_EXT)))
DSS_MRR_TI_DESIGN_OBJECTS   = $(addprefix $(PLATFORM_OBJDIR)/, $(DSS_MRR_TI_DESIGN_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# RTSC Configuration:
###################################################################################
dssMrrTIDesignRTSC: $(DSP_CFG)
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(C674_XSFLAGS) -o $(DSS_MRR_TI_DESIGN_CONFIGPKG) $(DSS_MRR_TI_DESIGN_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

###################################################################################
# Build the DSS MRR TI Design
###################################################################################
dssMrr: BUILD_CONFIGPKG=$(DSS_MRR_TI_DESIGN_CONFIGPKG)
dssMrr: C674_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt \
                        -i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft16x16/c64P	\
                        -i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P \
                        -i$(C674x_MATHLIB_INSTALL_PATH)/packages \

dssMrr: buildDirectories dssMrrTIDesignRTSC $(DSS_MRR_TI_DESIGN_OBJECTS)
	$(C674_LD) $(C674_LDFLAGS) $(DSS_MRR_TI_DESIGN_LOC_LIBS) $(DSS_MRR_TI_DESIGN_STD_LIBS) 			\
	-l$(DSS_MRR_TI_DESIGN_CONFIGPKG)/linker.cmd --map_file=$(DSS_MRR_TI_DESIGN_MAP) $(DSS_MRR_TI_DESIGN_OBJECTS) 	\
	$(PLATFORM_C674X_LINK_CMD) $(DSS_MRR_TI_DESIGN_CMD) $(C674_LD_RTS_FLAGS) -o $(DSS_MRR_DEMO_OUT)
	$(COPY_CMD) $(DSS_MRR_TI_DESIGN_CONFIGPKG)/package/cfg/$(DSS_MRR_TI_DESIGN_ROV_XS) $(DSS_MRR_TI_DESIGN_ROV_XS)
	@echo 'Built the DSS MRR TI Design [Preparing the BIN Format]'
	@echo '******************************************************************************'
	@echo 'Built the DSS MRR TI Design OUT and BIN Formats'
	@echo '******************************************************************************'

###################################################################################
# Cleanup the DSS MRR TI Design
###################################################################################
dssMrrClean:
	@echo 'Cleaning the DSS MRR TI Design Objects'
	@$(DEL) $(DSS_MRR_TI_DESIGN_OBJECTS) $(DSS_MRR_TI_DESIGN_MAP) $(DSS_MRR_DEMO_OUT) $(DSS_MRR_TI_DESIGN_BIN) $(DSS_MRR_TI_DESIGN_DEPENDS) $(DSS_MRR_TI_DESIGN_ROV_XS)
	@echo 'Cleaning the DSS MRR TI Design RTSC package'
	@$(DEL) $(DSS_MRR_TI_DESIGN_CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(DSS_MRR_TI_DESIGN_DEPENDS)

