//Arduino uno with base shield extension using FreeRTOS
/*  FreeRTOS V202112.00 */

#include <avr/io.h>
#include "FreeRTOS-Kernel/include/FreeRTOS.h" /* RTOS kernel functions. */
#include "task.h" /* RTOS task related API prototypes. */
#include "semphr.h" /* Semaphore related API prototypes. */



#define Idle_Priority   (tskIDLE_PRIORITY)


//tasks handler defined after the main
//wait for an RFIFD card to be presented
static void WaitingTask( void *pvParameters );
//read the RFID card and send it so the master device
static void RFIDTask( void *pvParameters );
//blink an led every second to show the system is alive
static void LEDTask( void *pvParameters );
//open the door(led) for 5 seconds when a valid card is presented
//and activate the ultrasound sensor
static void DoorTask( void *pvParameters );
//monitor the ultrasound sensor and close the door after 5 seconds, only 1 person allowed
//in the doorway at a time
static void UltrasoundTask( void *pvParameters );
static void CounterTask( void *pvParameters );

//constant to ease the reading....
const uint8_t doorLed = _BV(PD2);
const uint8_t greenLed = _BV(PD4);
const uint8_t ultrasoundSensor = _BV(PD6);
// UNO D8 is PB0
const uint8_t RFIDReader = _BV(PB0);
// UNO D10 is PB2 (typically SPI SS)
const uint8_t MasterDevice = _BV(PB2);



int main(void){
    DDRD |= (doorLed | greenLed); // PD2 and PD4 as outputs
    DDRD &= ~ultrasoundSensor;    // PD6 as input
    DDRB &= ~(RFIDReader | MasterDevice); // PB0 (D8) and PB2 (D10) as inputs

    // Create tasks
    xTaskCreate(
        WaitingTask,
        "WaitingTask",
        128,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    // Start scheduler.
    vTaskStartScheduler();

    return 0;
}

// Minimal implementation so the project links
static void WaitingTask( void *pvParameters ){
    (void)pvParameters;
    for(;;){
        // Simple heartbeat: toggle green LED every 500ms
        PORTD ^= greenLed;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
