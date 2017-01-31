#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "Audio.h"
#include "mp3dec.h"
#include "timer.h"
#include "mp3_data.h"

// Private variables
MP3FrameInfo mp3FrameInfo;
HMP3Decoder hMP3Decoder;

// Private function prototypes
static void AudioCallback(void *context,int buffer);
void init();

// Some macros
#define BUTTON		(GPIOA->IDR & GPIO_Pin_0)
#define LED_GREEN 	0
#define LED_ORANGE	1
#define LED_RED		2
#define LED_BLUE	3

int main(void) {
	init();
	int volume = 0;

	// Play mp3
	hMP3Decoder = MP3InitDecoder();
	InitializeAudio(Audio44100HzSettings);
	SetAudioVolume(0xCF);
	PlayAudioWithCallback(AudioCallback, 0);

	for(;;) {
		/*
		 * Check if user button is pressed
		 */
		if (BUTTON) {
			// Debounce
			Delay(10);
			if (BUTTON) {

				// Toggle audio volume
				if (volume) {
					volume = 0;
					SetAudioVolume(0xCF);
				} else {
					volume = 1;
					SetAudioVolume(0xAF);
				}


				while(BUTTON){};
			}
		}
	}

	return 0;
}

/*
 * Called by the audio driver when it is time to provide data to
 * one of the audio buffers (while the other buffer is sent to the
 * CODEC using DMA). One mp3 frame is decoded at a time and
 * provided to the audio driver.
 */
static void AudioCallback(void *context, int buffer) {
	static int16_t audio_buffer0[4096];
	static int16_t audio_buffer1[4096];
	static int led = 0;
	int offset, err;
	int outOfData = 0;
	static const char *read_ptr = mp3_data;
	static int bytes_left = MP3_DATA_SIZE;

	int16_t *samples;

	if (buffer) {
		samples = audio_buffer0;
	}
	else {
		samples = audio_buffer1;
	}
	switch (led){
		case LED_GREEN: 
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
			led = LED_ORANGE;
			break;
		case LED_ORANGE:
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15);
			led = LED_RED;
			break;
		case LED_RED: 
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15);
			led = LED_BLUE;
			break;
		default: 
			GPIO_SetBits(GPIOD, GPIO_Pin_15);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
			led = LED_GREEN;
			break;
	}
	offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	bytes_left -= offset;

	if (bytes_left <= 10000) {
		read_ptr = mp3_data;
		bytes_left = MP3_DATA_SIZE;
		offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	}

	read_ptr += offset;
	err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, &bytes_left, samples, 0);

	if (err) {
		/* error occurred */
		switch (err) {
		case ERR_MP3_INDATA_UNDERFLOW:
			outOfData = 1;
			break;
		case ERR_MP3_MAINDATA_UNDERFLOW:
			/* do nothing - next call to decode will provide more mainData */
			break;
		case ERR_MP3_FREE_BITRATE_SYNC:
		default:
			outOfData = 1;
			break;
		}
	} else {
		/* no error */
		MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
	}

	if (!outOfData) {
		ProvideAudioBuffer(samples, mp3FrameInfo.outputSamps);
	}
}

void init() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	// Enable full access to FPU (Should be done automatically in system_stm32f4xx.c):
	//SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access

	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Push-Pull: This is the output type that most people think of as "standard". When the output goes low, it is actively "pulled" to ground. 
	* Conversely, when the output is set to high, it is actively "pushed" toward Vcc
	*/
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;	// we want to configure all LED GPIO pins
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;										// we want the pins to be an output
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;										// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;									// this sets the GPIO modules clock speed
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;									// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStructure);												// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

	// ------ UART ------ //

	// Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);								// Enables the Low Speed APB ((APB1)advanced peripheral bus) peripheral clock.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}
