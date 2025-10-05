idf_component_register(SRCS "websocket_client.c"
                    INCLUDE_DIRS "include"
                    REQUIRES device_manager
                    PRIV_REQUIRES wifi_manager
                                  esp_websocket_client
                                  esp_event
                                  cjson
                                  log
                                  common)