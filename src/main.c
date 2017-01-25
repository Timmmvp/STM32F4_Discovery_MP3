#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "Audio.h"
#include "mp3dec.h"

// Private variables
volatile uint32_t time_var1, time_var2;
MP3FrameInfo mp3FrameInfo;
HMP3Decoder hMP3Decoder;

// Private function prototypes
static void AudioCallback(void *context,int buffer);
void Delay(volatile uint32_t nCount);
void init();

// External variables
extern const char mp3_data[];

// Some macros
#define MP3_SIZE	687348
#define BUTTON		(GPIOA->IDR & GPIO_Pin_0)

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

	int offset, err;
	int outOfData = 0;
	static const char *read_ptr = mp3_data;
	static int bytes_left = MP3_SIZE;

	int16_t *samples;

	if (buffer) {
		samples = audio_buffer0;
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	} else {
		samples = audio_buffer1;
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}

	offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	bytes_left -= offset;

	if (bytes_left <= 10000) {
		read_ptr = mp3_data;
		bytes_left = MP3_SIZE;
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
	USART_InitTypeDef USART_InitStructure;
	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	// Enable full access to FPU (Should be done automatically in system_stm32f4xx.c):
	//SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access

	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;	// refers to pins 12 up & till 15 as pins he's talking to and gives this value to the struct GPIO_InitStructure.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;										// sets pins 12 up & till 15 as outputs and gives this value to the struct GPIO_InitStructure.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;										// sets pushpull config for those pins and gives this value to the struct GPIO_InitStructure.
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;									// sets pinspeed and gives this value to the struct GPIO_InitStructure.
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;									// choose no pull resistor and gives this value to the struct GPIO_InitStructure.
	GPIO_Init(GPIOD, &GPIO_InitStructure);												// initialises port D with the values given to GPIO_InitStructure



	// ------ UART ------ //

	// Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);								// Enables the Low Speed APB ((APB1)advanced peripheral bus) peripheral clock.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;								// Refers to pins 5 & 6 as pins he's addressing and gives this value to the struct GPIO_InitStructure
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;									// sets pinspeed and gives this value to the struct GPIO_InitStructure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										// sets pins to alternating function (to use with peripheral USART) and gives this value to the struct GPIO_InitStructure
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;										// sets pins to pushpull config and gives this value to the struct GPIO_InitStructure
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;										// enables pull up resistor and gives this value to the struct GPIO_InitStructure
	GPIO_Init(GPIOD, &GPIO_InitStructure);												// initialises port D with the values given to the struct GPIO_InitStructure

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART1);							//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART1);							//

	// Conf
	USART_InitStructure.USART_BaudRate = 115200;										// sets BaudRate at 115200 and give this value to the struct USART_InitStructure.
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;							// sets the USART Word Length at 8 bits and gives the value to the struct USART_InitStructure
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								// sets the stop bit and gives this value to the struct USART_InitStructure
	USART_InitStructure.USART_Parity = USART_Parity_No;									// sets the Parity to NO and gives this value to the struct USART_InitStructure	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		// sets hardware flow control to NONE and gives this value to the struct USART_InitStructure
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;						// sets the USART mode to Transmitting and Receiving and gives the value to the struct USART_InitStructure
	USART_Init(USART2, &USART_InitStructure);											// initialises USART2 with the values given to the struct USART_InitStructure

	// Enable
	USART_Cmd(USART2, ENABLE);															// Enables USART
}

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}

	time_var2++;
}

/*
 * Delay a number of systick cycles
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;

	while(time_var1){};
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}
