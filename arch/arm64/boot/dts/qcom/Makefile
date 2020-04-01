ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SDM429) += rover-overlay.dtbo
rover-overlay.dtbo-base := rover.dtb
else
dtb-$(CONFIG_ARCH_SDM429) += rover.dtb
endif
always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
