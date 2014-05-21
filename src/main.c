#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "msprf24.h"

union {
  struct {
    uint8_t hh;
    uint8_t hl;
    uint8_t th;
    uint8_t tl;
    uint8_t crc;
  } val;
  uint8_t bytes[5];
} dht_data;

#define UART_DEBUG

#ifdef UART_DEBUG
char txbuf[256];
void uart_setup() {
  UCA0CTL0 = 0;
  UCA0CTL1 |= UCSWRST; // Put USCI in reset mode
  UCA0CTL1 = UCSSEL_2 | UCSWRST; // Use SMCLK, still reset
  UCA0MCTL = UCBRF_0 | UCBRS_6;
  UCA0BR0 = 131; // 9600 bauds
  UCA0BR1 = 6;
  UCA0CTL1 &= ~UCSWRST; // Normal mode
	P1SEL2 |= (BIT1 + BIT2); // Set pins for USCI
	P1SEL |= (BIT1 + BIT2);
}
void uart_send(int len) {
  int i;
  for(i = 0; i < len; i++) {
    while (UCA0STAT & UCBUSY);
    UCA0TXBUF = txbuf[i];
  }
}
#endif

void radio_setup() {
  uint8_t addr[] = {0xDE, 0xAD, 0x00, 0xBE, 0xEF};
  rf_crc = RF24_EN_CRC | RF24_CRCO;
  rf_addr_width = 5;
  rf_speed_power = RF24_SPEED_250KBPS | RF24_POWER_0DBM;
  rf_channel = 120;
  // Initialize and such
  msprf24_init();
  msprf24_set_pipe_packetsize(0, 0);
  msprf24_open_pipe(0, 1);
  // Wake up from deep sleep (if...)
  msprf24_standby();
  // Set TX and RX addresses (AutoACKs are sent to the TX address)
  w_tx_addr(addr);
  w_rx_addr(0, addr);
}

void verylongdelay() {
  TA0CCR0 = 4096;
  TA0CCTL0 |= CCIE;
  TA0CTL = TACLR;
  TA0CTL = TASSEL_1 | ID_3 | MC_1; // ACLK/8 (4096 hz), up to CCR0
  LPM3;
}

void __attribute__((interrupt (TIMER0_A0_VECTOR)))
verylongdelayisr() {
  TA0CTL = TACLR;
  LPM4_EXIT;
}

int main() {
  uint8_t buf[32];

  WDTCTL = WDTPW | WDTHOLD;
  DCOCTL = 0;
  BCSCTL1 = CALBC1_16MHZ;
  DCOCTL = CALDCO_16MHZ;
  BCSCTL3 |= LFXT1S_2;

  P1DIR |= BIT0;
  P1OUT &= ~BIT0;

  _BIS_SR(GIE);
#ifdef UART_DEBUG
  uart_setup();
#endif
  radio_setup();

  // flush RX just in case
  if (!(RF24_RX_EMPTY & msprf24_queue_state())) {
    flush_rx();
  }
  msprf24_activate_rx();
  LPM4;

  while (1) {
    if (rf_irq & RF24_IRQ_FLAGGED) {
      rf_irq &= ~RF24_IRQ_FLAGGED;
      msprf24_get_irq_reason();
    }
    if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending()) {
      uint8_t nextPayloadSize = r_rx_peek_payload_size();
      r_rx_payload(nextPayloadSize, buf);
      msprf24_irq_clear(RF24_IRQ_MASK);
      memcpy(dht_data.bytes, buf, nextPayloadSize);
#ifdef UART_DEBUG
      int t = (((dht_data.val.th&0x7f)<<8) + dht_data.val.tl)*((dht_data.val.th&0x80)?-1:1);
      int h = (dht_data.val.hh<<8) + (dht_data.val.hl);
      uart_send(sprintf(txbuf, "%d.%1d;%d.%1d\r\n", t/10, t%10, h/10, h%10));
#endif
    }
    LPM4;
  }

  return 0;
}
