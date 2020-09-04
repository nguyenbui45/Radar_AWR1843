###################################################################################
# MSS MRR TI Design
#
#  NOTE:
#      (C) Copyright 2017 Texas Instruments, Inc.
###################################################################################
.PHONY: mssMrr mssMrrClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c mss
vpath %.c common
vpath %.c $(MMWAVE_SDK_INSTALL_PATH)/ti/demo/utils \
          ./mss

###################################################################################
# Additional libraries which are required to build the MRR TI Design:
###################################################################################
MSS_MRR_TI_DESIGN_STD_LIBS = $(R4F_COMMON_STD_LIB)					\
   			-llibpinmux_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT) 		\
		   	-llibdma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
   			-llibcrc_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
   			-llibuart_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
		   	-llibgpio_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)  		\
   			-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
   			-llibedma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		    \
   			-llibmmwavelink_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)    \
   			-llibmmwave_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)        \
   			-llibcbuff_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)         	\
			-llibcli_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)    \
            -llibcanfd_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
MSS_MRR_TI_DESIGN_LOC_LIBS = $(R4F_COMMON_LOC_LIB)					\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/pinmux/lib 	   		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/uart/lib			\
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/dma/lib			\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/crc/lib		        \
		   	-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/gpio/lib		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib	        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwavelink/lib      \
	        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/edma/lib    		\
	        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/cbuff/lib    		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwave/lib          \
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/cli/lib        \
            -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/canfd/lib          \

###################################################################################
# MSS MRR TI Design
###################################################################################
MSS_MRR_CFG_PREFIX     = mss_mrr
MSS_MRR_TI_DESIGN_CFG       = mss/$(MSS_MRR_CFG_PREFIX).cfg

MSS_MRR_TI_DESIGN_ROV_XS    = $(MSS_MRR_CFG_PREFIX)_$(R4F_XS_SUFFIX).rov.xs
MSS_MRR_TI_DESIGN_CONFIGPKG = mss/mmw_configPkg_$(MMWAVE_SDK_DEVICE_TYPE)
MSS_MRR_TI_DESIGN_MAP       = $(MMWAVE_SDK_DEVICE_TYPE)_mrr_ti_design_mss.map
MSS_MRR_DEMO_OUT       = $(MMWAVE_SDK_DEVICE_TYPE)_mrr_ti_design_mss.$(R4F_EXE_EXT)
MSS_MRR_TI_DESIGN_BIN       = $(MMWAVE_SDK_DEVICE_TYPE)_mrr_ti_design_mss.bin
MSS_MRR_TI_DESIGN_CMD       = mss/mss_mrr_linker.cmd
MSS_MRR_TI_DESIGN_SOURCES   = mss_main.c mss_mrr_cli.c cfg.c
MSS_MRR_TI_DESIGN_DEPENDS   = $(addprefix $(PLATFORM_OBJDIR)/, $(MSS_MRR_TI_DESIGN_SOURCES:.c=.$(R4F_DEP_EXT)))
MSS_MRR_TI_DESIGN_OBJECTS   = $(addprefix $(PLATFORM_OBJDIR)/, $(MSS_MRR_TI_DESIGN_SOURCES:.c=.$(R4F_OBJ_EXT)))

###################################################################################
# RTSC Configuration:
###################################################################################
mssMrrTIDesignRTSC:
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(R4F_XSFLAGS) -o $(MSS_MRR_TI_DESIGN_CONFIGPKG) $(MSS_MRR_TI_DESIGN_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

###################################################################################
# Build the MRR TI Design
###################################################################################
mssMrr: BUILD_CONFIGPKG=$(MSS_MRR_TI_DESIGN_CONFIGPKG)
mssMrr: R4F_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt
mssMrr: buildDirectories mssMrrTIDesignRTSC $(MSS_MRR_TI_DESIGN_OBJECTS)
	$(R4F_LD) $(R4F_LDFLAGS) $(MSS_MRR_TI_DESIGN_LOC_LIBS) $(MSS_MRR_TI_DESIGN_STD_LIBS) 					\
	-l$(MSS_MRR_TI_DESIGN_CONFIGPKG)/linker.cmd --map_file=$(MSS_MRR_TI_DESIGN_MAP) $(MSS_MRR_TI_DESIGN_OBJECTS) 	\
	$(PLATFORM_R4F_LINK_CMD) $(MSS_MRR_TI_DESIGN_CMD) $(R4F_LD_RTS_FLAGS) -o $(MSS_MRR_DEMO_OUT)
	$(COPY_CMD) $(MSS_MRR_TI_DESIGN_CONFIGPKG)/package/cfg/$(MSS_MRR_TI_DESIGN_ROV_XS) $(MSS_MRR_TI_DESIGN_ROV_XS)
	@echo 'Built the MSS MRR TI Design [Preparing the BIN Format]'
	@echo '******************************************************************************'
	@echo 'Built the MSS MRR TI Design OUT & BIN Formats'
	@echo '******************************************************************************'

###################################################################################
# Cleanup the MRR TI Design
###################################################################################
mssMrrClean:
	@echo 'Cleaning the MSS MRR TI Design Objects'
	@$(DEL) $(MSS_MRR_TI_DESIGN_OBJECTS) $(MSS_MRR_TI_DESIGN_MAP) $(MSS_MRR_DEMO_OUT) $(MSS_MRR_TI_DESIGN_BIN) $(MSS_MRR_TI_DESIGN_DEPENDS) $(MSS_MRR_TI_DESIGN_ROV_XS)
	@echo 'Cleaning the MSS MRR TI Design RTSC package'
	@$(DEL) $(MSS_MRR_TI_DESIGN_CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(MSS_MRR_TI_DESIGN_DEPENDS)

