
#
# DRIVERS
#


# PADS
ifeq '$(CONFIG_PADS_ENABLED)' '1'
PULP_CFLAGS += -DCONFIG_PADS_ENABLED=1
ifneq '$(padframe/version)' ''
PULP_LIB_FC_SRCS_rt += drivers/pads/pads-v$(padframe/version).c
endif
endif


# UDMA

ifneq '$(udma/version)' ''
PULP_CFLAGS += -D__RT_UDMA_COPY_ASM=1
PULP_LIB_FC_SRCS_rt     += drivers/udma/udma-v$(udma/archi).c
PULP_LIB_FC_ASM_SRCS_rt += drivers/udma/udma-v$(udma/archi)_asm.S
endif


# HYPER

ifeq '$(CONFIG_HYPER_ENABLED)' '1'
ifneq '$(udma/hyper/version)' ''
PULP_CFLAGS += -D__RT_HYPER_COPY_ASM=1
PULP_LIB_FC_SRCS_rt += drivers/hyper/hyperram-v$(udma/hyper/version).c
PULP_LIB_FC_ASM_SRCS_rt += drivers/hyper/hyperram-v$(udma/hyper/version)_asm.S
endif
endif


# CAM

ifeq '$(CONFIG_CAM_ENABLED)' '1'
ifneq '$(udma/cpi/version)' ''
PULP_LIB_FC_SRCS_rt += drivers/cpi/cpi-v1.c
endif
endif


# I2C

ifeq '$(CONFIG_I2C_ENABLED)' '1'
ifneq '$(udma/i2c/version)' ''
PULP_CFLAGS += -D__RT_I2C_COPY_ASM=1
PULP_LIB_FC_SRCS_rt += drivers/i2c/i2c-v$(udma/i2c/version).c drivers/i2c/i2c-v$(udma/i2c/version)_asm.c
endif
endif

# PWM

ifeq '$(pulp_chip_family)' 'gap'
ifeq '$(CONFIG_PWM_ENABLED)' '1'
PULP_LIB_FC_CFLAGS += -DRT_CONFIG_PWM_ENABLED
PULP_LIB_FC_SRCS_rt += drivers/pwm/pwm.c
PULP_LIB_FC_ASM_SRCS_rt += drivers/pwm/pwm_asm.S
endif
endif



# SPIM

ifeq '$(CONFIG_SPIM_ENABLED)' '1'
ifneq '$(udma/spim/version)' ''
PULP_CFLAGS += -D__RT_SPIM_COPY_ASM=1
PULP_LIB_FC_SRCS_rt += drivers/spi/spim-v$(udma/spim/version).c
PULP_LIB_FC_ASM_SRCS_rt += drivers/spi/spim-v$(udma/spim/version)_asm.S
endif
endif


# GPIO

ifeq '$(pulp_chip_family)' 'gap'
ifeq '$(CONFIG_GPIO_ENABLED)' '1'
PULP_FC_CFLAGS += -DRT_CONFIG_GPIO_ENABLED
ifneq '$(gpio/version)' ''
PULP_LIB_FC_SRCS_rt += drivers/gpio/gpio-v$(gpio/version).c
#PULP_LIB_FC_ASM_SRCS_rt += drivers/gpio/gpio-v$(gpio/version)_asm.S
endif
endif
endif



# UART

ifeq '$(CONFIG_UART_ENABLED)' '1'
ifneq '$(udma/uart/version)' ''
PULP_LIB_FC_SRCS_rt += drivers/uart/uart.c
endif
endif

