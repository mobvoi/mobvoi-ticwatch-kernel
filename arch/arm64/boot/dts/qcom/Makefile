ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SDM429) += rover-overlay.dtbo
dtbo-$(CONFIG_ARCH_SDM429) += rubyfish-overlay.dtbo
dtbo-$(CONFIG_ARCH_SDM429) += ricefish-overlay.dtbo
rover-overlay.dtbo-base := rover.dtb
rubyfish-overlay.dtbo-base := rubyfish.dtb
ricefish-overlay.dtbo-base := ricefish.dtb
else
dtb-$(CONFIG_ARCH_SDM429) += rover.dtb
dtb-$(CONFIG_ARCH_SDM429) += rubyfish.dtb
dtb-$(CONFIG_ARCH_SDM429) += ricefish.dtb
endif
always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
