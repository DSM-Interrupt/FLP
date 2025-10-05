idf_component_register(SRCS "web_server.c"
                    INCLUDE_DIRS "include"
                    REQUIRES wifi_manager device_manager websocket_client
                    PRIV_REQUIRES esp_http_server
                                  esp_netif
                                  lwip
                                  log
                                  esp_wifi)