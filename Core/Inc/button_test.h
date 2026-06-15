/*
 * button_test.h  –  PB3 push-button bring-up self-test (HydraSense)
 *
 * Verifies wiring, pull-up, debounce and the three press tiers used by the
 * production firmware (short / long >=3 s / very-long >=10 s).
 *
 * Compiled only when BUTTON_TEST_MODE is defined (zero flash cost otherwise).
 * Requires WS2812B_Init() and Buzzer_Init() to have run first, i.e. call
 * Button_Test() after App_Init().
 */
#ifndef BUTTON_TEST_H
#define BUTTON_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BUTTON_TEST_MODE
void Button_Test(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_TEST_H */
