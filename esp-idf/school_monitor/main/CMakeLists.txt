idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "main.c" 
                            "misc/global_data.c"
                            "misc/wifi_setup.c" 
                            "misc/mqtt_setup.c"
                            "misc/ota_setup.c"
                            "inmp441/inmp441_setup.cpp" 
                            "bme280/bme280_setup.c" 
                            "bme280/bme280.c" 
                            "dsm501a/dsm501a_setup.c"
                            "neo6m/neo6m_setup.c"
                    INCLUDE_DIRS "neo6m" "bme280" "dsm501a" "inmp441" "misc" "."
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
