
idf_component_register(SRCS "kzq.c" "modbus_params.c" "master.c" "spiffs.c" "gatts.c" "dht11.c" "mesh.c" "uart.c" "socket.c" "iic.c" 
                    INCLUDE_DIRS ""
                    REQUIRES freemodbus spiffs  mcommon mconfig mwifi mlink
)
