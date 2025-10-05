idf_component_register(SRCS "nvs_storage.c"
                    INCLUDE_DIRS "include"
                    # nvs_flash.h를 사용하기 위해 nvs_flash 의존성 추가
                    PRIV_REQUIRES common log nvs_flash)