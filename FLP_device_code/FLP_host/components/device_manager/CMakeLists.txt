idf_component_register(SRCS "device_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES common
                    PRIV_REQUIRES nvs_storage log)