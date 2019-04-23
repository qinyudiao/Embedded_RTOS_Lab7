/**
 * @file 
 * @author Riley Wood and Jeageun Jung
 * @brief ADC driver for the TM4C123G.
 *        Provides interfaces for collecting single samples
 *        or a series at a given sampling frequency.
 *        Does not allow for sampling of more than one channel at
 *        any given time.
 *        Timer 2 is reserved for this driver.
 * 
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/**
 * @brief Configure an ADC channel for continuous sampling.
 *        Retrieve measurements from this channel with ADC_In().
 * 
 * @param channelNum The channel to set up
 * @return int 0 on success, -1 on failure.
 */
int ADC_Init(uint32_t channelNum);

/**
 * @brief Returns the most recent sample collected
 *        by the channel configured in ADC_Init(...)
 * 
 * This function uses busy-wait for the ADC sampling to be done
 *
 * @return uint16_t The conversion result
 */
uint16_t ADC_In(void);

/**
 * @brief Kick off collection of a sequence of samples to be passed to a user-provided handler.
 *        The ADC and Timer will be configured to collect samples at frequency fs.
 * 
 * @param channelNum ADC channel to sample
 * @param fs Sampling frequency
 * @param handler Function which will be passed each sample as it is collected.
 * @return int 0 on success, -1 on failure.
 */
int ADC_Collect(uint32_t channelNum, uint32_t fs, void (*handler)(unsigned long));


#endif
