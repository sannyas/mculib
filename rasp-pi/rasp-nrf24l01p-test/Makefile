CSRC += main.c
CSRC += nrf24l01p_platform_pi.c
CSRC += $(MCULIB_PATH)/chip/nrf24l01p/nrf24l01p.c
INC  += $(MCULIB_PATH)/chip/nrf24l01p

CFLAGS += $(addprefix -I,$(INC)) -DNRF24L01P_DEBUG

all:
	gcc $(CSRC) $(CFLAGS) -DMCULIB_DEBUG -o nrf24l01p -lwiringPi 
