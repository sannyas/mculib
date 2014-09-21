#include "nrf24l01p.h"

/**
 * These functions is implementation-dependent, which should
 * be rewrite in different platform.
 *
 * @{
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <wiringPi.h>

int spi_fd;
#define SPI_DEVICE	"/dev/spidev0.0"

/**
 * @brief Read and write data using SPI.
 *
 * MSBit first.
 */
/**
 * @brief Execute command.
 *
 * @param cmd	Command code, see header file.
 * @param data	This is a result-value parameter.
 * @param len	The length of data.
 */

void nrf24l01p_doCommand( uint8_t cmd, uint8_t const *data_in, uint8_t *data_out, uint8_t len ){
	uint8_t idx;
	struct spi_ioc_transfer spi_trans[2];	
	int ret;

	memset( spi_trans, 0, sizeof(spi_trans) );
	spi_trans[0].tx_buf = (unsigned long)&cmd;
	spi_trans[0].len = 1;
	spi_trans[1].tx_buf = (unsigned long)data_in;
	spi_trans[1].rx_buf = (unsigned long)data_out;
	spi_trans[1].len = len;

	if( len == 0 ){
		ret = ioctl( spi_fd, SPI_IOC_MESSAGE(1), spi_trans );
	}
	else{
		ret = ioctl( spi_fd, SPI_IOC_MESSAGE(2), spi_trans );
	}
	if( ret < 0 ){
		perror( "doCommand" );
	}
}

/**
 * @brief Set Chip Enable.
 * 
 * @param en	0 or 1.
 */
void nrf24l01p_chipEnable( uint8_t en ){
	if( en ){
		digitalWrite( 0, 1 );
	}
	else{
		digitalWrite( 0, 0 );
	}
}

/**
 * @brief Platform-dependent initialization, about SPI interface.
 */
void nrf24l01p_platformInit( void ){
	__u8	mode, lsb, bits;
	__u32	speed;
	int ret;

	spi_fd = open( "/dev/spidev0.0", O_RDWR );
	if( spi_fd < 0 ){
		fprintf( stderr, "Can't open spi device <%s> ", SPI_DEVICE );
		perror( "" );
		exit( 1 );
	}

	ret = wiringPiSetup();
	if( ret < 0 ){
		perror( "wringPi setup error" );	
		exit( 0 );
	}
	pinMode( 0, OUTPUT );
	digitalWrite( 0, 0 );

	uint8_t spiMode;
	if (ioctl (spi_fd, SPI_IOC_WR_MODE, &spiMode)         < 0) return;
	if (ioctl (spi_fd, SPI_IOC_RD_MODE, &spiMode)         < 0) return;

	uint8_t spiBPW;
	if (ioctl (spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0) return;
	if (ioctl (spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spiBPW) < 0) return;

	if (ioctl (spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)   < 0) return;
	speed = 5000;
	if (ioctl (spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)   < 0) return;

}

void nrf24l01p_delay_ms( uint16_t n ){
	usleep((n)*1000);
}

void nrf24l01p_delay_us( uint16_t n ){
	usleep(n);
}

/*@}*/
