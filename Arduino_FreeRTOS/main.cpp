#include "FreeRTOS.h"
#include "task.h" /* RTOS task related API prototypes. */
#include "semphr.h" /* Semaphore related API prototypes. */
#include <avr/io.h>

//Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ), where configMAX_PRIORITIES is defined within FreeRTOSConfig.h. 
// Low priority numbers denote low priority tasks. The idle task has priority zero (tskIDLE_PRIORITY). 

#define mainLED_TASK_PRIORITY   (tskIDLE_PRIORITY)

static SemaphoreHandle_t xEventRedSemaphore = NULL;

//tasks handler defined after the main
static void vRedBlinkLed(void* pvParameters);
static void vGreenBlinkLed(void* pvParameters);

//constant to ease the reading....
const uint8_t redLed   = _BV(PD2);
const uint8_t greenLed = _BV(PD3);

static volatile uint8_t myCounter = 0;

int main(void)
{
    xEventRedSemaphore = xSemaphoreCreateBinary();


    DDRD |= (redLed | greenLed); // PD2 and PD3 as outputs

    // Create task #1
    TaskHandle_t redBlink_handle;
    xTaskCreate                     //documented here: https://www.freertos.org/a00125.html
    (
        vRedBlinkLed,               //pointer function to the handler
        (const char*)"redBlink",    //naming the task
        configMINIMAL_STACK_SIZE,   //stack size
        NULL,                       //parameters of the handler
        1U,                         //priority
        &redBlink_handle            //address of task handler
    );

    // Create task #2
    TaskHandle_t greenBlink_handle;
    xTaskCreate
    (
        vGreenBlinkLed,
        (const char*)"greenBlink",
        configMINIMAL_STACK_SIZE,
        NULL,
        1U,
        &greenBlink_handle
    );

    // Start scheduler.
    vTaskStartScheduler();

    return 0;
}



/**************************************************************************//**
 * \fn static void vGreenBlinkLed(void* pvParameters)
 *
 * \brief toggle the green led.
 *
 * \param[in]   pvParameters
 ******************************************************************************/

 static void vGreenBlinkLed(void* pvParameters)
{
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
        myCounter++;
        if(myCounter%4 == 0){
            xSemaphoreGive(xEventRedSemaphore);
            myCounter = 0;
        }
        PORTD ^= greenLed; //PD3 on the micro controller is linked to D3 on the shield
        vTaskDelayUntil(&xLastWakeUpTime, 250/portTICK_PERIOD_MS);  //passive Delay

    }
}

/**************************************************************************//**
 * \fn static void vRedBlinkLed(void* pvParameters)
 *
 * \brief  toggle the red led
 *
 * \param[in]   pvParameters
 ******************************************************************************/

 static void vRedBlinkLed(void* pvParameters)
{
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
        /* Block until the semaphore is 'given'. **NOTE:**
           A semaphore is used for example purposes. In a real application it might
           be preferable to use a direct to task notification, which will be faster
           and use less RAM. */
        xSemaphoreTake( xEventRedSemaphore, portMAX_DELAY );
        PORTD ^= redLed; //PD2 on the micro controller is linked to D2 on the shield
    }
}

