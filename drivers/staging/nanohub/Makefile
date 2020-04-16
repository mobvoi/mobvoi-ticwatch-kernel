#
# Makefile for nanohub
#

obj-$(CONFIG_NANOHUB) += nanohub.o
nanohub-y := main.o bl.o comms.o custom_app_event.o
nanohub-$(CONFIG_NANOHUB_MAX1726X) += max1726x_fuelgauge.o
nanohub-$(CONFIG_NANOHUB_SPI) += spi.o
nanohub-$(CONFIG_NANOHUB_I2C) += i2c.o
