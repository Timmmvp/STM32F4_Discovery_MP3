#include "Audio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include <stdlib.h>

static void WriteRegister(uint8_t address, uint8_t value);
static void StartAudioDMAAndRequestBuffers();
static void StopAudioDMA();

static AudioCallbackFunction *CallbackFunction;
static void *CallbackContext;
static int16_t * volatile NextBufferSamples;
static volatile int NextBufferLength;
static volatile int BufferNumber;
static volatile bool DMARunning;

void InitializeAudio(int plln, int pllr, int i2sdiv, int i2sodd) {
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Intitialize state.
	CallbackFunction = NULL;
	CallbackContext = NULL;
	NextBufferSamples = NULL;
	NextBufferLength = 0;
	BufferNumber = 0;
	DMARunning = false;

	// Turn on peripherals.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);					// Turns on the peripheral for GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);					// Turns on the peripheral for GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);					// Turns on the peripheral for GPIOC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);					// Turns on the peripheral for GPIOD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);					// Turns on the peripheral for DMA1

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);					// Turns on the peripheral for I2C communication
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);					// Turns on the peripheral for SPI communication
	
	/* This TypeDef is a structure defined in the
	 * ST's library and it contains all the properties
	 * the corresponding peripheral has, such as output mode,
	 * pullup / pulldown resistors etc.
	 * 
	 * These structures are defined for every peripheral so 
	 * every peripheral has it's own TypeDef. The good news is
	 * they always work the same so once you've got a hang
	 * of it you can initialize any peripheral.
	 * 
	 * The properties of the periperals can be found in the corresponding
	 * header file e.g. stm32f4xx_gpio.h and the source file stm32f4xx_gpio.c
	 */
	
	// Configure reset pin.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;								// we want to configure PD4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;							// we want the pin to be an output 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						// this sets the GPIO modules clock speed
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;						// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStructure);									// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

	// Configure I2C SCL and SDA pins.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;					// we want to configure PB6 and PB9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						// this sets the GPIO modules clock speed
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							// we want the pins to be in Alternating Function
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;							// this sets the pin type to open drain (as opposed to push / pull)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;						// this sets the pullup / pulldown resistor to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStructure);									// this finally passees all the values to the GPIO_Init function which takes care of setting the corresponding bits.

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);					// configuarates pin 6 for the use of I2C communication.
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);					// configuarates pin 9 for the use of I2C communication.

	// Configure I2S MCK, SCK, SD pins.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;	// we want to configure PC7 | PC10 | PC12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						// this sets the GPIO modules clock speed
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							// we want the pins to be in Alternating Function
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;						// this sets the pullup / pulldown resistor to be inactive 
	GPIO_Init(GPIOC, &GPIO_InitStructure);									// this finally passees all the values to the GPIO_Init function which takes care of setting the corresponding bits.

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3);					// configurates pin 7 of port C for the use of I2S communication.
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);				// configurates pin 10 of port C for the use of I2S communication.
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);				// configurates pin 12 of port C for the use of I2S communication.

	// Configure I2S WordSelect pin.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;								// we want to configure PA4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;						// this sets the GPIO modules clock speed
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							// we want the pins to be in Alternating Function
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;							// this sets teh pin type to push / pull (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;						// this sets the pullup / pulldown  resistor to be inactive
	GPIO_Init(GPIOA, &GPIO_InitStructure);									// this finally passees all the values to the GPIO_Init function which takes care of setting the corresponding bits.

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);					// configuarates PA4 to use as WordSelect pin.

	// Reset the codec.
	GPIOD ->BSRRH = 1 << 4;
	for (volatile int i = 0; i < 0x4fff; i++) {
		__asm__ volatile("nop");
	}
	GPIOD ->BSRRL = 1 << 4;

	// Reset I2C.
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);			
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	// Configure I2C.
	uint32_t pclk1 = 42000000;

	I2C1 ->CR2 = pclk1 / 1000000; // Configure frequency and disable interrupts and DMA.
	I2C1 ->OAR1 = I2C_OAR1_ADDMODE | 0x33;
    
	// Configure I2C speed in standard mode.
	const uint32_t i2c_speed = 100000;
	int ccrspeed = pclk1 / (i2c_speed * 2);
	if (ccrspeed < 4) {
		ccrspeed = 4;
	}
	I2C1 ->CCR = ccrspeed;
	I2C1 ->TRISE = pclk1 / 1000000 + 1;

	I2C1 ->CR1 = I2C_CR1_ACK | I2C_CR1_PE; // Enable and configure the I2C peripheral.

	// Configure codec.
	WriteRegister(0x02, 0x01); // Keep codec powered off.
	WriteRegister(0x04, 0xaf); // SPK always off and HP always on.  what is SPK and what is HP?

	WriteRegister(0x05, 0x81); // Clock configuration: Auto detection.
	WriteRegister(0x06, 0x04); // Set slave mode and Philips audio standard.

	SetAudioVolume(0xff);

	// Power on the codec.
	WriteRegister(0x02, 0x9e);

	// Configure codec for fast shutdown.
	WriteRegister(0x0a, 0x00); // Disable the analog soft ramp.
	WriteRegister(0x0e, 0x04); // Disable the digital soft ramp.

	WriteRegister(0x27, 0x00); // Disable the limiter attack level.
	WriteRegister(0x1f, 0x0f); // Adjust bass and treble levels.

	WriteRegister(0x1a, 0x0a); // Adjust PCM volume level.
	WriteRegister(0x1b, 0x0a);

/* I2S is a popular 3 wire serial bus standard protocol developed by Philips for transmission of 2 channel (stereo) Pulse Code Modulation digital data,
 * where each audio sample is sent MSB first. I2S signals consist of a bit-clock, Left/Right Clock (also is often referred to as the Word Select)
 * and alternating left and right channel data. This protocol can be compared to synchronous serial ports in TDM mode with 2 timeslots (or channels) active.
 * This multiplexed protocol requires only 1 data path to send/receive 2 channels of digital audio information.
*/
	// Disable I2S.
	SPI3 ->I2SCFGR = 0;

	// I2S clock configuration
	RCC ->CFGR &= ~RCC_CFGR_I2SSRC; // PLLI2S clock used as I2S clock source.
	RCC ->PLLI2SCFGR = (pllr << 28) | (plln << 6);

	// Enable PLLI2S and wait until it is ready.
	RCC ->CR |= RCC_CR_PLLI2SON;
	while (!(RCC ->CR & RCC_CR_PLLI2SRDY ))
		;

	// Configure I2S.
	SPI3 ->I2SPR = i2sdiv | (i2sodd << 8) | SPI_I2SPR_MCKOE;
	SPI3 ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1
			| SPI_I2SCFGR_I2SE; // Master transmitter, Philips mode, 16 bit values, clock polarity low, enable.

}

void AudioOn() {
	WriteRegister(0x02, 0x9e);
	SPI3 ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1
			| SPI_I2SCFGR_I2SE; // Master transmitter, Philips mode, 16 bit values, clock polarity low, enable.
}

void AudioOff() {
	WriteRegister(0x02, 0x01);
	SPI3 ->I2SCFGR = 0;
}

void SetAudioVolume(int volume) {
	WriteRegister(0x20, (volume + 0x19) & 0xff);
	WriteRegister(0x21, (volume + 0x19) & 0xff);
}

void OutputAudioSample(int16_t sample) {
	while (!(SPI3 ->SR & SPI_SR_TXE ))
		;
	SPI3 ->DR = sample;
}

void OutputAudioSampleWithoutBlocking(int16_t sample) {
	SPI3 ->DR = sample;
}

void PlayAudioWithCallback(AudioCallbackFunction *callback, void *context) {
	StopAudioDMA();

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	NVIC_SetPriority(DMA1_Stream7_IRQn, 4);

	SPI3 ->CR2 |= SPI_CR2_TXDMAEN; // Enable I2S TX DMA request.

	CallbackFunction = callback;
	CallbackContext = context;
	BufferNumber = 0;

	if (CallbackFunction)
		CallbackFunction(CallbackContext, BufferNumber);
}

void StopAudio() {
	StopAudioDMA();
	SPI3 ->CR2 &= ~SPI_CR2_TXDMAEN; // Disable I2S TX DMA request.
	NVIC_DisableIRQ(DMA1_Stream7_IRQn);
	CallbackFunction = NULL;
}

void ProvideAudioBuffer(void *samples, int numsamples) {
	while (!ProvideAudioBufferWithoutBlocking(samples, numsamples))
		__asm__ volatile ("wfi");
}

bool ProvideAudioBufferWithoutBlocking(void *samples, int numsamples) {
	if (NextBufferSamples)
		return false;

	NVIC_DisableIRQ(DMA1_Stream7_IRQn);

	NextBufferSamples = samples;
	NextBufferLength = numsamples;

	if (!DMARunning)
		StartAudioDMAAndRequestBuffers();

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	return true;
}

static void WriteRegister(uint8_t address, uint8_t value) {
	while (I2C1 ->SR2 & I2C_SR2_BUSY )
		;

	I2C1 ->CR1 |= I2C_CR1_START; // Start the transfer sequence.
	while (!(I2C1 ->SR1 & I2C_SR1_SB ))
		; // Wait for start bit.

	I2C1 ->DR = 0x94;
	while (!(I2C1 ->SR1 & I2C_SR1_ADDR ))
		; // Wait for master transmitter mode.
	I2C1 ->SR2;

	I2C1 ->DR = address; // Transmit the address to write to.
	while (!(I2C1 ->SR1 & I2C_SR1_TXE ))
		; // Wait for byte to move to shift register.

	I2C1 ->DR = value; // Transmit the value.

	while (!(I2C1 ->SR1 & I2C_SR1_BTF ))
		; // Wait for all bytes to finish.
	I2C1 ->CR1 |= I2C_CR1_STOP; // End the transfer sequence.
}

static void StartAudioDMAAndRequestBuffers() {
	// Configure DMA stream.
	DMA1_Stream7 ->CR = (0 * DMA_SxCR_CHSEL_0 ) | // Channel 0
			(1 * DMA_SxCR_PL_0 ) | // Priority 1
			(1 * DMA_SxCR_PSIZE_0 ) | // PSIZE = 16 bit
			(1 * DMA_SxCR_MSIZE_0 ) | // MSIZE = 16 bit
			DMA_SxCR_MINC | // Increase memory address
			(1 * DMA_SxCR_DIR_0 ) | // Memory to peripheral
			DMA_SxCR_TCIE; // Transfer complete interrupt
	DMA1_Stream7 ->NDTR = NextBufferLength;
	DMA1_Stream7 ->PAR = (uint32_t) &SPI3 ->DR;
	DMA1_Stream7 ->M0AR = (uint32_t) NextBufferSamples;
	DMA1_Stream7 ->FCR = DMA_SxFCR_DMDIS;
	DMA1_Stream7 ->CR |= DMA_SxCR_EN;

	// Update state.
	NextBufferSamples = NULL;
	BufferNumber ^= 1;
	DMARunning = true;

	// Invoke callback if it exists to queue up another buffer.
	if (CallbackFunction)
		CallbackFunction(CallbackContext, BufferNumber);
}

static void StopAudioDMA() {
	DMA1_Stream7 ->CR &= ~DMA_SxCR_EN; // Disable Direct Memory Access stream.
	while (DMA1_Stream7 ->CR & DMA_SxCR_EN )
		; // Wait for DMA stream to stop.

	DMARunning = false;
}

void DMA1_Stream7_IRQHandler() {
	DMA1 ->HIFCR |= DMA_HIFCR_CTCIF7; // Clear interrupt flag.

	if (NextBufferSamples) {
		StartAudioDMAAndRequestBuffers();
	} else {
		DMARunning = false;
	}
}
