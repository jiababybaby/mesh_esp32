
idf_component_register(SRCS "gatts.c" "spiffs.c" "mesh.c" "uart.c" "socket.c" "znwg.c" 
                    INCLUDE_DIRS ""
                    REQUIRES spiffs mcommon mconfig mwifi mlink
)
