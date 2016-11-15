// BzBox Firmware Application - Alpha - October 2016
// OSBH Development
// Created by: Peter Naylor 
//
// Project Location:
// https://github.com/pnaylor1982/
//
// LOTS of help and reference from Particle.io community:
// ricckas7, ScruffR, peekay123, kennethlimcp, etc.
// 
//
// Libraries can be found here:
// https://github.com/rickkas7/photonAudio
// https://github.com/mhaack/SparkFun_BME280
// https://github.com/kennethlimcp/spark-adxl345
// https://github.com/sparkfun/Battery_Babysitter
//
// TODO:
// 		- Deep sleep
//		- Classification algorithm
//		- Memory card backup for audio samples
//		- Sleep mode for peripherals
//



#include "Particle.h"
#include "SparkFunBME280.h"
#include "SparkFunBQ27441.h"
#include "adxl345.h"
#include "adc_hal.h"
#include "gpio_hal.h"
#include "pinmap_hal.h"
#include "pinmap_impl.h"

// System threading is required for this project
SYSTEM_THREAD(ENABLED);

// Class instances
ADXL345 accel;
BME280 OutSense;
BME280 InSense;
TCPClient client;

//const unsigned int data_rate = 15000;
//const unsigned int serial_rate = 5000;
const unsigned int BATTERY_CAPACITY = 2000; // e.g. 850mAh battery

// This is the pin connected to the INT1 pin on the ADXL362 breakout
// Don't use D0 or D1; they are used for I2C to the PowerShield battery monitor
const int ACCEL_INT_PIN = D2;

// This is the name of the Particle event to publish for battery or movement detection events
// It is a private event.
//const char *eventName = "accel";

// Various timing constants
const unsigned long MAX_TIME_TO_PUBLISH_MS = 20000; // Only stay awake for 20 seconds trying to connect to the cloud and publish
const unsigned long TIME_AFTER_PUBLISH_MS = 3000; // After publish, wait 4 seconds for data to go out
const unsigned long TIME_AFTER_BOOT_MS = 500; // At boot, wait 1 seconds before going to publish
//const unsigned long TIME_PUBLISH_BATTERY_SEC = 4 * 60 * 60; // every 4 hours, send a battery update
const unsigned long TIME_PUBLISH_BATTERY_SEC = 15 * 60; // every 4 hours, send a battery update

// Stuff for the finite state machine
enum State { BOOT_WAIT_STATE, PUBLISH_STATE, PUBLISH_WAIT_STATE, AWS_CONNECT, AWS_STREAM, AWS_FINISH, SLEEP_STATE};
State state = BOOT_WAIT_STATE;

//void buttonHandler(system_event_t event, int data); // forward declaration


// 2048 is a good size for this buffer. This is the number of samples; the number of bytes is twice
// this, but only half the buffer is sent a time, and then each pair of samples is averaged, so
// this results in 1024 byte writes from TCP, which is optimal.
// This uses 4096 bytes of RAM, which is also reasonable.
const size_t SAMPLE_BUF_SIZE = 2048;

// This is the pin the microphone is connected to.
const int SAMPLE_PIN = A0;

// The audio sample rate. The minimum is probably 8000 for minimally acceptable audio quality.
// Not sure what the maximum rate is, but it's pretty high.
const long SAMPLE_RATE = 6300;

// If you don't hit the setup button to stop recording, this is how long to go before turning it
// off automatically. The limit really is only the disk space available to receive the file.
const unsigned long MAX_RECORDING_LENGTH_MS = 10000;

// This is the IP Address and port that the audioServer.js node server is running on (AWS)
IPAddress serverAddr = IPAddress(52,36,158,55); //52,36,158,55 : 192,168,0,34
int serverPort = 7123;


uint16_t samples[SAMPLE_BUF_SIZE];

unsigned long recordingStart;

int count = 0;
char myStr[64];
unsigned long timer_data;
unsigned long timer_serial = 0;
int data_item = 0;

// BME280 values
float tempC_in;
float pressure_in;
float altitude_in;
float RH_in;
float tempC_out;
float pressure_out;
float altitude_out;
float RH_out;
// LiPo values
unsigned int soc;
unsigned int volts;
int current;
int power;
int health;

unsigned long stateTime = 0;
int awake = 0;
int qrn = 0;

bool p_once = false;

// ADCDMA - Class to use Photon ADC in DMA Mode
class ADCDMA {
public:
	ADCDMA(int pin, uint16_t *buf, size_t bufSize);
	virtual ~ADCDMA();

	void start(size_t freqHZ);
	void stop();

private:
	int pin;
	uint16_t *buf;
	size_t bufSize;
};

// Helpful post:
// https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fcortex%5fmx%5fstm32%2fstm32f207%20ADC%2bTIMER%2bDMA%20%20Poor%20Peripheral%20Library%20Examples&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=6249

ADCDMA::ADCDMA(int pin, uint16_t *buf, size_t bufSize) : pin(pin), buf(buf), bufSize(bufSize) {
}

ADCDMA::~ADCDMA() {

}

void ADCDMA::start(size_t freqHZ) {

    // Using Dual ADC Regular Simultaneous DMA Mode 1

	// Using Timer3. To change timers, make sure you edit all of:
	// RCC_APB1Periph_TIM3, TIM3, ADC_ExternalTrigConv_T3_TRGO

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Set the pin as analog input
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    HAL_Pin_Mode(pin, AN_INPUT);

	// Enable the DMA Stream IRQ Channel
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 60000000UL = 60 MHz Timer Clock = HCLK / 2
	// Even low audio rates like 8000 Hz will fit in a 16-bit counter with no prescaler (period = 7500)
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (60000000UL / freqHZ) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T3_TRGO
	TIM_Cmd(TIM3, ENABLE);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	// DMA2 Stream0 channel0 configuration
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buf;
	DMA_InitStructure.DMA_PeripheralBaseAddr =  0x40012308; // CDR_ADDRESS; Packed ADC1, ADC2;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = bufSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	// Don't enable DMA Stream Half / Transfer Complete interrupt
	// Since we want to write out of loop anyway, there's no real advantage to using the interrupt, and as
	// far as I can tell, you can't set the interrupt handler for DMA2_Stream0 without modifying
	// system firmware because there's no built-in handler for it.
	// DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

	DMA_Cmd(DMA2_Stream0, ENABLE);

	// ADC Common Init
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ADC1 configuration
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC2 configuration - same
	ADC_Init(ADC2, &ADC_InitStructure);

	//
	ADC_RegularChannelConfig(ADC1, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Enable ADCs
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	ADC_SoftwareStartConv(ADC1);
}

void ADCDMA::stop() {
	// Stop the ADC
	ADC_Cmd(ADC1, DISABLE);
	ADC_Cmd(ADC2, DISABLE);

	DMA_Cmd(DMA2_Stream0, DISABLE);

	// Stop the timer
	TIM_Cmd(TIM3, DISABLE);
}

ADCDMA adcDMA(SAMPLE_PIN, samples, SAMPLE_BUF_SIZE);

// End ADCDMA

void setup() {

  //***Driver settings********************************//
InSense.settings.commInterface = I2C_MODE;
InSense.settings.I2CAddress = 0x77;

OutSense.settings.commInterface = I2C_MODE;
OutSense.settings.I2CAddress = 0x76;


//***Operation settings*****************************//

//runMode can be:
//  0, Sleep mode
//  1 or 2, Forced mode
//  3, Normal mode
InSense.settings.runMode = 3; //Forced mode
OutSense.settings.runMode = 3; //Forced mode

//tStandby can be:
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
InSense.settings.tStandby = 0;
OutSense.settings.tStandby = 0;

//filter can be off or number of FIR coefficients to use:
//  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8
//  4, coefficients = 16
InSense.settings.filter = 0;
OutSense.settings.filter = 0;

//tempOverSample can be:
//  0, skipped
//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
InSense.settings.tempOverSample = 1;
OutSense.settings.tempOverSample = 1;

//pressOverSample can be:
//  0, skipped
//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
InSense.settings.pressOverSample = 1;
OutSense.settings.pressOverSample = 1;

//humidOverSample can be:
//  0, skipped
//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
InSense.settings.humidOverSample = 1;
OutSense.settings.humidOverSample = 1;

//delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.         Serial.begin(57600);

Serial.begin(9600);
delay(5000);

Serial.println("Setup");

Serial.print("Starting BME280_in... result of .begin(): 0x");
//Calling .begin() causes the settings to be loaded
Serial.println(InSense.begin(), HEX);

Serial.print("Starting BME280_out... result of .begin(): 0x");
//Calling .begin() causes the settings to be loaded
Serial.println(OutSense.begin(), HEX);

// Initialization for the battery monitor
if (!lipo.begin())
{
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
//    while (1) ;
}

Serial.println("Connected to BQ27441!");
lipo.setCapacity(BATTERY_CAPACITY); // Configure BQ27441 to assume a 1000 mAh battery

// Initialization for accelerometer
delay(250);
accel.powerOn();
delay(250);

Serial.println("Setting up accelerometer");

accel.setRangeSetting(2);			// 2g range
accel.setActivityThreshold(75);	// out of 255
accel.setInactivityThreshold(25);	// out of 255
accel.setActivityX(1);			
accel.setActivityY(1);
accel.setActivityZ(1);

// Debug messages
qrn = accel.getActivityThreshold();
Serial.print("Activity threshold: ");
Serial.println(qrn);

qrn = accel.isActivityXEnabled();
Serial.print("x-en?  ");
Serial.println(qrn);

qrn = accel.isActivityYEnabled();
Serial.print("y-en?  ");
Serial.println(qrn);

qrn = accel.isActivityZEnabled();
Serial.print("z-en?  ");
Serial.println(qrn);

delay(100);
// Map the AWAKE bit to INT1 so activity can wake up the Photon
accel.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN);
accel.setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1);

			// Activity timer is not used because when inactive we go into sleep mode; it automatically wakes after 1 sample
			// Set inactivity timer to 250 or 5 seconds at 50 samples/sec
				///accel.setTimeInactivity(250);

			// Enable loop operation (automatically move from activity to inactivity without having to clear interrupts)
			// Enable activity and inactivity detection in reference mode (automatically accounts for gravity)
			// uint8_t linkLoop, bool inactRef, bool inactEn, bool actRef, bool actEn
				///accel.writeActivityControl(accel.LINKLOOP_LOOP, true, true, true, true);

			// Enable measuring mode with auto-sleep
			// bool extClock, uint8_t lowNoise, bool wakeup, bool autosleep, uint8_t measureMode
				///accel.writePowerCtl(false, false, false, true, accel.MEASURE_MEASUREMENT);

//			qrn = accel.isInterruptEnabled(ADXL345_INT_ACTIVITY_BIT);
//			Serial.println(qrn);

pinMode(D7, OUTPUT);

}


void loop() {
	uint8_t status;
	uint16_t *sendBuf = NULL;

	switch(state) {

	case BOOT_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_BOOT_MS) {
			// To publish the battery stats after boot, set state to PUBLISH_STATE
			// To go to sleep immediately, set state to SLEEP_STATE
			state = PUBLISH_STATE;
			p_once = false;
		}
		break;

	case PUBLISH_STATE:

		if (!p_once) {
		// BME280 reads
		tempC_in = InSense.readTempC();
		pressure_in = InSense.readFloatPressure();
		altitude_in = InSense.readFloatAltitudeMeters();
		RH_in = InSense.readFloatHumidity();

		tempC_out = OutSense.readTempC();
		pressure_out = OutSense.readFloatPressure();
		altitude_out = OutSense.readFloatAltitudeMeters();
		RH_out = OutSense.readFloatHumidity();

		// BQ27441 Battery Manager calls
		soc = lipo.soc(); // Read state-of-charge (in %)
		volts = lipo.voltage(); // Read voltage (in mV)
		current = lipo.current(AVG); // Read average current (in mA)
		power = lipo.power(); // Read power consumption (in mW)
		health = lipo.soh(); // Read state-of-health (in %)

		// Debug printouts
			Serial.println("__IN_;_OUT__");
			Serial.print("temp (C): ");
			Serial.print(tempC_in);
			Serial.print(" ; ");
			Serial.println(tempC_out);
			Serial.print("Pressure (kPa): ");
			Serial.print(pressure_in);
			Serial.print(" ; ");
			Serial.println(pressure_out);
			Serial.print("Altitude (m): ");
			Serial.print(altitude_in);
			Serial.print(" ; ");
			Serial.println(altitude_out);
			Serial.print("RH (%): ");
			Serial.print(RH_in);
			Serial.print(" ; ");
			Serial.println(RH_out);

			Serial.print("State of Charge/Health: ");
			Serial.print(soc);
			Serial.print(", ");
			Serial.println(health);
			Serial.print("mV, mA, mW: ");
			Serial.print(volts);
			Serial.print(", ");
			Serial.print(current);
			Serial.print(", ");
			Serial.println(power);

			p_once = true;
		}
		
		if (Particle.connected()) {
			Serial.println("publish");
			char data[128];
			//float cellVoltage = batteryMonitor.getVCell();
			//float stateOfCharge = batteryMonitor.getSoC();
			//snprintf(data, sizeof(data), "%d,%.02f,%.02f", awake, cellVoltage, stateOfCharge);
			//snprintf(data, sizeof(data), "%d", awake);
			snprintf(data, sizeof(data), "%.01f,%.01f,%.01f,%.01f,%.01f,%.01f,%d,%d,%d,%d", tempC_in, tempC_out, pressure_in, pressure_out, RH_in, RH_out, soc, volts, current, power);

			//Particle.publish(eventName, data, 60, PRIVATE);
			Particle.publish("wakey wakey", data);
			accel.getInterruptSource();

			// Wait for the publish to go out
			stateTime = millis();
			state = PUBLISH_WAIT_STATE;
		}
		else {
			// Haven't come online yet
			if (millis() - stateTime >= MAX_TIME_TO_PUBLISH_MS) {
				// Took too long to publish, just go to sleep
				state = SLEEP_STATE;
			}
		}
		break;

	case PUBLISH_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_PUBLISH_MS) {
			state = AWS_CONNECT;
		}
		break;

	case AWS_CONNECT:
		// Ready to connect to the server via TCP
		if (client.connect(serverAddr, serverPort)) {
			// Connected
			adcDMA.start(SAMPLE_RATE);

			Serial.println("aws starting");

			recordingStart = millis();
			digitalWrite(D7, HIGH);

			state = AWS_STREAM;
		}
		else {
			Serial.println("failed to connect to server");
			state = SLEEP_STATE;
		}
		break;

	case AWS_STREAM:
		if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_HTIF0)) {
		    DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_HTIF0);
		    sendBuf = samples;
		}
		if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0)) {
		    DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
		    sendBuf = &samples[SAMPLE_BUF_SIZE / 2];
		}

		if (sendBuf != NULL) {
			// There is a sample buffer to send

			// Average the pairs of samples
			for(size_t ii = 0, jj = 0; ii < SAMPLE_BUF_SIZE / 2; ii += 2, jj++) {
				uint32_t sum = (uint32_t)sendBuf[ii] + (uint32_t)sendBuf[ii + 1];
				sendBuf[jj] = (uint16_t)(sum / 2);
			}

			// Send here. We're actually sending 1/4 of the samples here, 1/2 of the
			// samples buffer, and we've averaged each pair of samples, but the samples
			// are 16 bits and client.write() takes bytes, so only / 2 here
			int count = client.write((uint8_t *)sendBuf, SAMPLE_BUF_SIZE / 2);
			if (count == SAMPLE_BUF_SIZE / 2) {
				// Success
			}
			else
			if (count == -16) {
				// TCP Buffer full
				Serial.printlnf("buffer full, discarding");
			}
			else {
				// Error
				Serial.printlnf("error writing %d", count);
				state = AWS_FINISH;
			}
		}

		if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS) {
			state = AWS_FINISH;
		}
		break;

	case AWS_FINISH:
		digitalWrite(D7, LOW);
		adcDMA.stop();
		client.stop();
		Serial.println("aws stopping");
		state = SLEEP_STATE;
		break;

	case SLEEP_STATE:
		// Sleep
		System.sleep(ACCEL_INT_PIN, RISING, TIME_PUBLISH_BATTERY_SEC);

		///awake = ((accel.readStatus() & accel.STATUS_AWAKE) != 0);

		// check to see if reason for waking was xlrometer disturbance, or timer
		if(accel.getInterruptSource()){
			Serial.println("xlrometer disturbance~!");
			state = BOOT_WAIT_STATE;
		}
		else{
			state = BOOT_WAIT_STATE;
		}
		stateTime = millis();
		break;
	}

}