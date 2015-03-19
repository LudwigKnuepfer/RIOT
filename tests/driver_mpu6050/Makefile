APPLICATION = driver_mpu6050
include ../Makefile.tests_common

# Define default mappings for some boards:
ifneq (,$(filter samr21-xpro,$(BOARD)))
  export TEST_I2C ?= I2C_0
  export TEST_HW_ADDR ?= MPU6050_HW_ADDR_HEX_68
endif

FEATURES_REQUIRED = periph_i2c periph_gpio

USEMODULE += mpu6050
USEMODULE += vtimer

ifneq (,$(TEST_I2C))
  CFLAGS += -DTEST_I2C=$(TEST_I2C)
else
  # set random default
  CFLAGS += -DTEST_I2C=I2C_0
endif
ifneq (,$(TEST_HW_ADDR))
  CFLAGS += -DTEST_HW_ADDR=$(TEST_HW_ADDR)
else
  # set random default
  CFLAGS += -DTEST_HW_ADDR=MPU6050_HW_ADDR_HEX_68
endif

include $(RIOTBASE)/Makefile.include
