/*  Copyright (c) 2015 B. Sidhipong <bsidhipong@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include <avr/common.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <spi.h>
#include "nrf24l01.h"

#if !(defined(NRF24L01_HARDWARE_DEVDUINO_V2) || \
      defined(NRF24L01_HARDWARE_RFTOY_V1))
#pragma GCC error "nRF24L01+ radio implementation must be defined."
#endif

#ifndef cbi
#define cbi(sfr,bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr,bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#if defined(NRF24L01_HARDWARE_DEVDUINO_V2)
#define __ce_low sbi(PORTB,PB0)
#define __ce_high cbi(PORTB,PB0)
#define __csn_high sbi(PORTD,PD7)
#define __csn_low cbi(PORTD,PD7)
#elif defined(NRF24L01_HARDWARE_RFTOY_V1)
#define __ce_low sbi(PORTC,PC3)
#define __ce_high cbi(PORTC,PC3)
#define __csn_high sbi(PORTC,PC2)
#define __csn_low cbi(PORTC,PC2)
#endif
#define ce(state) __ce_##state
#define csn(state) __csn_##state

static uint8_t status = 0;

void nrf24l01_init( void )
{
	register uint8_t content;
	/* Set up chip-select and chip-enable. */
#if defined(NRF24L01_HARDWARE_DEVDUINO_V2)
	sbi(DDRB, DDB0);
	sbi(DDRD, DDD7);
#elif defined(NRF24L01_HARDWARE_RFTOY_V1)
	sbi(DDRC, DDC2);
	sbi(DDRC, DDC3);
#endif
#if defined(NRF24L01_HARDWARE_DEVDUINO_V2) | \
    defined(NRF24L01_HARDWARE_RFTOY_V1)
	/* Set up external interrupt for the radio (PD2, INT0, active low). */
	cbi(DDRD, DDD2);
	sbi(EICRA, ISC01);
	sbi(EIMSK, INT0);
#endif
	ce(low);
	csn(low);
	/* Configure the nRF24L01 with default parameters:
	 * • No interrupts
	 * • 16-bit CRC */
	nrf24l01_write_register(CONFIG,
			((0 << MASK_RX_DR) |
			 (0 << MASK_TX_DS) |
			 (0 << MASK_MAX_RT) |
			 (1 << EN_CRC) |
			 (1 << CRC0) |
			 (0 << PWR_UP) |
			 (0 << PRIM_RX)));
	/* 7.5.2 Auto Retransmission (ART)
	 * The auto retransmission is a function that retransmits a packet if an ACK packet is not received. It is used
	 * at the PTX side in an auto acknowledgement system. You can set up the number of times a packet is
	 * allowed to be retransmitted if a packet is not acknowledged with the ARC bits in the SETUP_RETR register.
	 * PTX enters RX mode and waits a time period for an ACK packet each time a packet is transmitted. The
	 * time period the PTX is in RX mode is based on the following conditions:
	 * • Auto Retransmit Delay (ARD) elapsed or
	 * • No address match within 250µs or
	 * • After received packet (CRC correct or not) if address match within 250µs
	 *
	 * Set up automatic-retransmit with the following parameters:
	 * ARD = 1250uS
	 * ARC = 15 */
	nrf24l01_write_register(SETUP_RETR, (0b0100 << ARD) | 0b1111);
	/* 7.4.1 Static and Dynamic Payload Length
	 * The Enhanced ShockBurst™ provides two alternatives for handling payload lengths, static and dynamic.
	 * The default alternative is static payload length. With static payload length all packets between a transmitter
	 * and a receiver have the same length. Static payload length is set by the RX_PW_Px registers on the
	 * receiver side. The payload length on the transmitter side is set by the number of bytes clocked into the
	 * TX_FIFO and must equal the value in the RX_PW_Px register on the receiver side
	 *
	 * Dynamic Payload Length(DPL) is an alternative to static payload length.DPL enables the transmitter to
	 * send packets with variabel payload length to the receiver. This means for a system with different payload
	 * lenghts it is not necessary to scale the packet length to the longest payload.
	 * With DPL feature the nRF24L01 can decode the payload length of the received packet automatically
	 * instead of using the RX_PW_Px registers. The MCU can read the length of the received payload by using
	 * the R_RX_PL_WID command.
	 *
	 * In order to enable DPL the EN_DPL bit in the FEATURE register must be set. In RX mode the DYNPD register
	 * has to be set. A PTX that transmits to a PRX with DPL enabled must have the DPL_P0 bit in DYNPD
	 * set. */
	content = nrf24l01_read_register(FEATURE);
	nrf24l01_write_register(FEATURE, content | _BV(EN_DPL) | _BV(EN_DYN_ACK));
	/* The R_RX_PL_WID, W_ACK_PAYLOAD, and W_TX_PAYLOAD_NOACK features registers are initially in a deactivated
	 * state; a write has no effect, a read only results in zeros on MISO. To activate these registers, use the
	 * ACTIVATE command followed by data 0x73. Then they can be accessed as any other register in nRF24L01. Use
	 * the same command and data to deactivate the registers again.*/
	content = nrf24l01_read_register(FEATURE);
	if (0 == content) { /* need to send ACTIVATE command */
		spi_send(ACTIVATE);
		spi_send(0x73);
		nrf24l01_write_register(FEATURE, content | _BV(EN_DPL) | _BV(EN_DYN_ACK));
	}
	/* 6.4 PA control
	 * The PA control is used to set the output power from the nRF24L01 power
	 * amplifier (PA). In TX mode PA control has four programmable steps,
	 * see Table 14.
	 *
	 * The PA control is set by the RF_PWR bits in the RF_SETUP register.
	 * ==================================================
	 * RF_PWR | RF output | DC current
	 *        | power     | consumption
	 * --------------------------------------------------
	 *   11   |    0dBm   | 11.3mA
	 *   10   |   -6dBm   | 9.0mA
	 *   01   |  -12dBm   | 7.5mA
	 *   00   |  -18dBm   | 7.0mA */
	nrf24l01_set_pa_control(POWER_MAX);
	nrf24l01_write_register(DYNPD, (_BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0)));
	/* clear RX_DR, TX_DS, and MAX_RT bits by writing 1 to the corresponding bits in STATUS register */
	nrf24l01_write_register(STATUS, (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)));
	nrf24l01_set_channel(1);
	status = spi_exchange(FLUSH_TX);
	status = spi_exchange(FLUSH_RX);
	csn(high);
}

/* 6.4 PA control
 * The PA control is used to set the output power from the nRF24L01 power
 * amplifier (PA). In TX mode PA * control has four programmable steps,
 * see Table 14.
 *
 * The PA control is set by the RF_PWR bits in the RF_SETUP register.
 * ==================================================
 * RF_PWR | RF output | DC current
 *        | power     | consumption
 * --------------------------------------------------
 *   11   |    0dBm   | 11.3mA
 *   10   |   -6dBm   | 9.0mA
 *   01   |  -12dBm   | 7.5mA
 *   00   |  -18dBm   | 7.0mA */
uint8_t nrf24l01_set_pa_control( uint8_t power )
{
	uint8_t setup = nrf24l01_read_register(RF_SETUP);
	setup &= ~(0b11 << RF_PWR);
	setup |= power;
	status = nrf24l01_write_register(RF_SETUP, setup);
	return status;
}

uint8_t nrf24l01_set_data_rate( uint8_t rate )
{
	uint8_t setup = nrf24l01_read_register(RF_SETUP);
	setup &= ~(_BV(RF_DR)|_BV(RF_DR_LOW));
	setup |= rate;
	status = nrf24l01_write_register(RF_SETUP, setup);
	return status;
}

uint8_t nrf24l01_set_channel( uint8_t channel )
{
	status = nrf24l01_write_register(RF_CH, channel);
	return status;
}

/* 8.3.1 SPI Commands
 * The SPI commands are shown in Table 16.  Every new command must be started by a high to low transition on CSN.
 *
 * In parallel to the SPI command word applied on the MOSI pin, the STATUS register is shifted serially out on the MISO pin.
 *
 * The serial shifting SPI commands is in the following format:
 * <command word: most-significant bit to least-significant bit (one byte)>
 * <data bytes: least-significant byte to most-significant byte, most-significant bit in each byte first> */
uint8_t nrf24l01_write_register( uint8_t reg_no, uint8_t content )
{
	csn(low);
	status = spi_exchange(W_REGISTER | (REGISTER_MASK & reg_no));
	spi_send(content);
	csn(high);
	return status;
}

uint8_t nrf24l01_write_multibyte_register( uint8_t reg_no, const uint8_t *content, uint8_t count )
{
	csn(low);
	status = spi_exchange(W_REGISTER | (REGISTER_MASK & reg_no));
	while (count--) spi_send(*content++);
	csn(high);
	return status;
}

uint8_t nrf24l01_read_register( uint8_t reg_no )
{
	register uint8_t data;
	csn(low);
	status = spi_exchange(R_REGISTER | (REGISTER_MASK & reg_no));
	data = spi_exchange(NOP);
	csn(high);
	return data;
}

uint8_t nrf24l01_read_multibyte_register( uint8_t reg_no, uint8_t *content, uint8_t count )
{
	csn(low);
	status = spi_exchange(R_REGISTER | (REGISTER_MASK & reg_no));
	while (count--) *content++ = spi_exchange(NOP);
	csn(high);
	return status;
}

uint8_t nrf24l01_transmit( const void *payload, uint8_t payload_size )
{
	uint8_t transmission_status;
	nrf24l01_transition(PRIMARY_TRANSMIT);
	/* 6.1.5 TX mode
	 * The TX mode is an active mode where the nRF24L01 transmits a packet. To enter this mode, the nRF24L01 must have the
	 * PWR_UP bit set high, PRIM_RX bit set low, a payload in the TX FIFO and, a high pulse on the CE for more than 10μs. */
	csn(low);
	status = spi_exchange(W_TX_PAYLOAD_NOACK);
	while (payload_size--) spi_send(*(uint8_t *)payload++);
	csn(high);
	/* starts the transmission... */
	ce(high); _delay_us(15); ce(low);
	_delay_us(150); /* 130μs TX settling time */
	do {
		uint8_t observe_tx = nrf24l01_read_register(OBSERVE_TX);
	} while (0 == (status & (_BV(TX_DS) | _BV(MAX_RT))));
	transmission_status = nrf24l01_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	/* At this point, because CE=0 and TX FIFO is empty, the radio should be in STANDBY_I state.
	 * We transition to POWER DOWN state to conserve battery. */
	nrf24l01_transition(POWER_DOWN);
	/* return the original payload_size if data was transmitted successfully, otherwise return 0 */
	return (transmission_status & _BV(TX_DS)) ? payload_size : 0;
}

uint8_t nrf24l01_receive( const void *payload, uint8_t limit )
{
	uint8_t payload_width;
	csn(low);
	spi_exchange(R_RX_PL_WID);
	payload_width = spi_exchange(NOP);
	status = spi_exchange(R_RX_PAYLOAD);
	/* adjust the read size to payload width or limit, whichever is smaller */
	if (payload_width < limit) limit = payload_width;
	while (limit--) *(uint8_t *)payload++ = spi_exchange(NOP);
	csn(high);
	return limit;
}

uint8_t nrf24l01_is_data_available( )
{
	return (0 == (nrf24l01_read_register(FIFO_STATUS) & _BV(RX_EMPTY)));
}

uint8_t nrf24l01_transition( nrf24l01_state_transition_t state )
{
	uint8_t config_register = nrf24l01_read_register(CONFIG);
	/* Check wheter we are currently in POWER DOWN state */
	if (0 == (config_register & _BV(PWR_UP))) {
		if (state > POWER_DOWN) nrf24l01_write_register(CONFIG, config_register | _BV(PWR_UP));
		_delay_ms(5); /* 1.5ms start-up time */
		/* radio now in STANDBY-I state */
	}
	switch( state ) {
		case POWER_DOWN:
			nrf24l01_write_register(CONFIG, config_register & ~_BV(PWR_UP));
			ce(low);
			break;
		case PRIMARY_TRANSMIT:
			nrf24l01_write_register(CONFIG, config_register & ~_BV(PRIM_RX));
			break;
		case PRIMARY_RECEIVE:
			nrf24l01_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
			nrf24l01_write_register(CONFIG, config_register | _BV(PRIM_RX));
			ce(high);
			_delay_us(150); /* 130μs RX settling time */
			break;
		case STANDBY_I:
			ce(low);
			break;
		case STANDBY_II:
			/* nRF24L01 will only transition into this state if:
			 * • PRIM_RX = 0
			 * • CE = 1
			 * • TX FIFO is empty (not under our control) */
			nrf24l01_write_register(CONFIG, config_register & ~_BV(PRIM_RX));
			ce(high);
			_delay_us(150); /* 130μs TX settling time */
			break;
	}
	return status;
}
