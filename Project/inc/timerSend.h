#ifndef __TIMER_SEND_H
#define __TIMER_SEND_H

#define TIM4_RETRY_INTERVAL_MS          15

void sleep_ms(uint16_t _ms);
void sleep_8us(uint8_t _delay);
void delaySend(uint8_t _base, uint16_t _ms, uint8_t *pMsg);

#endif // __TIMER_SEND_