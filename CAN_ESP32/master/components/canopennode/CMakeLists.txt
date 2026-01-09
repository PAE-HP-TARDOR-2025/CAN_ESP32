idf_component_register(
    SRCS
        "CANopen.c"
        "CO_driver.c"
        "OD.c"
        "CANopenNode_ESP32.c"
       
        "CANopen_LSS_Master.c"
        "fw_master_update.c"
        
        # --- 301 (CANopen application layer) ---
        "301/CO_fifo.c"
        "301/CO_Emergency.c"
        "301/CO_HBconsumer.c"
        "301/CO_ODinterface.c"
        "301/CO_NMT_Heartbeat.c"
        "301/CO_PDO.c"
        "301/CO_SDOclient.c"
        "301/CO_SDOserver.c"
        "301/CO_SYNC.c"
        "301/CO_TIME.c"
        "301/crc16-ccitt.c"

        # --- 303 (LED indicators) ---
        "303/CO_LEDs.c"

        # --- 304 (Safety) ---
        "304/CO_SRDO.c"
        "304/CO_GFC.c"

        # --- 305 (LSS services) ---
        "305/CO_LSSmaster.c"
        "305/CO_LSSslave.c"

        # --- 309 gateway ---
        "309/CO_gateway_ascii.c"

        # --- Storage ---
        "storage/CO_storage.c"
        #"storage/CO_storageEeprom.c"

        # --- Extra (tracing, performance, etc.) ---
        "extra/CO_trace.c"

        # --- example ---
        #"example/CO_driver_blank.c"
        "example/CO_storageBlank.c"
        #"example/main_blank.c"
        #"example/OD.c"

    INCLUDE_DIRS
        "."
        "./301"
        "./303"
        "./304"
        "./305"
        "./309"
        "./storage"
        "./extra"
        "./example"

    REQUIRES
        driver driver esp_timer nvs_flash log    # si usas el CAN driver del ESP32
    
)

