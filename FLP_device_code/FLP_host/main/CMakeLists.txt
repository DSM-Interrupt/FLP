idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES common
                                  device_manager
                                  gps_reader
                                  lora_driver
                                  nvs_storage
                                  web_server
                                  websocket_client
                                  wifi_manager
                                  driver
                                  cjson
                                  log)