idf_component_register(SRCS "csrc/a8i2cGateway.c"
                       INCLUDE_DIRS "csrc"
                       REQUIRES i2cdev freertos log)