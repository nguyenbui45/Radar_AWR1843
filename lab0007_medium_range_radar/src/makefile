###################################################################################
# MRR TI Design makefile
###################################################################################
ifeq ($(CCS_MAKEFILE_BASED_BUILD), 1)
ifeq ($(OS),Windows_NT)
include $(MMWAVE_SDK_INSTALL_PATH)/scripts/windows/setenv.mak
else
include $(MMWAVE_SDK_INSTALL_PATH)/scripts/unix/setenv.mak
endif
endif


include $(MMWAVE_SDK_INSTALL_PATH)/ti/common/mmwave_sdk.mak
include ./mss/mss_mrr.mak
include ./dss/dss_mrr.mak

MRR_TI_DESIGN_BIN = $(MMWAVE_SDK_DEVICE_TYPE)_mrr_ti_design.bin

###################################################################################
# Standard Targets which need to be implemented by each mmWave SDK module. This
# plugs into the release scripts.
###################################################################################
.PHONY: all clean mrrTiDesignClean mrrTiDesign
.NOTPARALLEL:


# Clean: This cleans all the objects
mrrTiDesignClean: mssMrrClean dssMrrClean
	@$(DEL) $(MRR_TI_DESIGN_BIN)

# This builds the MRR TI Design
mrrTiDesign: mssMrr dssMrr
    
# Clean: This cleans all the objects
clean: mrrTiDesignClean
    
# This builds all the supported demos
all: mrrTiDesign
	@$(GENERATE_METAIMAGE) $(MRR_TI_DESIGN_BIN) $(SHMEM_ALLOC) $(MSS_MRR_DEMO_OUT) $(XWR18XX_RADARSS_IMAGE_BIN) $(DSS_MRR_DEMO_OUT)
# Help: This displays the MAKEFILE Usage.
help:
	@echo '****************************************************************************************'
	@echo '* Makefile Targets for the DEMO '
	@echo 'mrrTiDesign         -> Builds the mmw Demo (both MSS and DSS binaries)'
	@echo 'mrrTiDesignClean    -> Cleans the mmw Demo (both MSS and DSS binaries)'
	@echo 'mssMrr         -> Builds the MSS binary for the mmw Demo'
	@echo 'mssMrrClean    -> Cleans the MSS binary for mmw Demo '
	@echo 'dssMrr         -> Builds the DSS binary for the mmw Demo'
	@echo 'dssMrrClean    -> Cleans the DSS binary for mmw Demo '
	@echo '****************************************************************************************'

