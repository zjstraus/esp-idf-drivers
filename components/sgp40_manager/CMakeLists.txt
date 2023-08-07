idf_component_register(SRCS "sgp40_manager.c" "sensirion_gas_index_algorithm.c"
        INCLUDE_DIRS "include"
        REQUIRES sgp40 esp_event)
