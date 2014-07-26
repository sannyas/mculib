CSRC += main.c
CSRC += nrf24l01p_platform_pi.c
CSRC += $(MCULIB_PATH)/device/nrf24l01p/nrf24l01p.c
INC  += $(MCULIB_PATH)/include

CFLAGS += $(addprefix -I,$(INC))

all:
	gcc $(CSRC) $(CFLAGS) -DMCULIB_DEBUG -o nrf24l01p -lwiringPi 
