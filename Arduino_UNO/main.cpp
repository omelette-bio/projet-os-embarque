// Arduino uno with base shield extension using FreeRTOS
/*  FreeRTOS V202112.00 */

#define BAUD 9600
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS-Kernel/include/FreeRTOS.h" /* RTOS kernel functions. */
#include "task.h"                             /* RTOS task related API prototypes. */
#include "semphr.h"                           /* Semaphore related API prototypes. */
#include <Wire.h>
#include <string.h>
#include <util/setbaud.h>

#include <util/delay.h>

#define Idle_Priority (tskIDLE_PRIORITY)

static void RFIDReadTask(void *pvParameters);
static void ActivateActuatorTask(void *pvParameters);

void receiveEvent(int howMany);
void requestEvent();

// Buffer pour l'UART
#define UID_SIZE 16

// Adresse I2C de l'Arduino, à modifier pour simuler différentes salles
// 0x42 -> entre, 0x43 -> entre pas, 0x44 -> entre pas
#define SLAVE_ADRESS 0x43

// Les actionneurs
const uint8_t blueLED       = _BV(PD5);
const uint8_t buzzer        = _BV(PD6);

// Variables globales
unsigned char current_uid[UID_SIZE]         = {0};
volatile uint8_t activate_actuator_ok       = 0;
volatile uint8_t activate_actuator_not_ok   = 0;



class RfidTask
{
private:
    volatile uint8_t _buffer[UID_SIZE];
    volatile uint8_t _writeIndex;
    volatile bool _dataReady;
    volatile bool _isReading;

public:
    RfidTask() : _writeIndex(0), _dataReady(false), _isReading(false) {}

    // Initialisation de l'UART
    void init()
    {
        // Configuration du Baud Rate (utilisant util/setbaud.h)
        UBRR0H = UBRRH_VALUE;
        UBRR0L = UBRRL_VALUE;

        #if USE_2X
                UCSR0A |= (1 << U2X0);
        #else
                UCSR0A &= ~(1 << U2X0);
        #endif

        // Activer TX et RX, et l'interruption de réception (RXCIE0)
        UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

        // Format de frame: 8 data, 1 stop bit, pas de parité
        UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    }

    // Méthode appelée par l'ISR pour traiter l'octet entrant
    void processIncomingByte(uint8_t byte)
    {
        const uint8_t STX = 0x02; // Start of Text
        const uint8_t ETX = 0x03; // End of Text

        if (byte == STX)
        {
            // Début de trame : on reset le buffer
            _writeIndex = 0;
            _isReading = true;
            _dataReady = false; // Nouvelle lecture en cours, l'ancienne n'est plus valide
        }
        else if (byte == ETX)
        {
            // Fin de trame
            if (_isReading)
            {
                _buffer[_writeIndex] = '\0'; // Null-terminate pour string C
                _dataReady = true;
                _isReading = false;
            }
        }
        else if (_isReading)
        {
            // Lecture des données
            if (_writeIndex < (UID_SIZE - 1))
            {
                _buffer[_writeIndex++] = byte;
            }
            else
            {
                // Buffer overflow : trame invalide
                _isReading = false;
                _writeIndex = 0;
            }
        }
    }

    // Vérifie si un nouvel UID est disponible
    bool isUidAvailable()
    {
        if (_dataReady)
        {
            _dataReady = false; // Reset du flag après lecture
            return true;
        }
        return false;
    }

    // Récupère l'UID (pointeur vers le buffer interne)
    // Utile pour la future transmission I2C
    const uint8_t *getUidBuffer()
    {
        return (const uint8_t *)_buffer;
    }

    // Récupère la longueur de l'UID
    uint8_t getUidLength()
    {
        return _writeIndex;
    }
};

RfidTask rfidReader;

ISR(USART_RX_vect)
{
    uint8_t receivedByte = UDR0; // Lire le registre de données UART
    rfidReader.processIncomingByte(receivedByte);
}

int main(void)
{
    DDRD    = 0b11111111;
    EICRA   = 0b00000010;
    EIMSK   |= (1 << INT0);

    // uart_init(); // UART Matériel (PC)
    rfidReader.init();
    Wire.begin(SLAVE_ADRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    sei();

    // Create tasks
    xTaskCreate(RFIDReadTask, "RFIDReadTask", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(ActivateActuatorTask, "ActivateActuatorTask", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
    // xTaskCreate(CheckPassageTask, "CheckPassageTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    return 0;
}

void receiveEvent(int howMany)
{
    if (!Wire.available()) return;

    char command = Wire.read();

    if (command == 'O') activate_actuator_ok = 1;
    if (command == 'N') activate_actuator_not_ok = 1;
    
}

void requestEvent()
{
    Wire.write((const uint8_t *)current_uid, UID_SIZE);
    for (int i = 0; i < UID_SIZE; i++) current_uid[i] = 0;
}

static void RFIDReadTask(void *pvParameters)
{
    while(1)
    {
        if (rfidReader.isUidAvailable())
        {
            const uint8_t* uid = rfidReader.getUidBuffer();
            if (uid[0] == 0xFF) continue;
            uint8_t len = rfidReader.getUidLength();

            for (int i=0; i<len; i++) current_uid[i] = uid[i];
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void ActivateActuatorTask(void *pvParameters)
{
    while(1)
    {
        if (activate_actuator_ok)
        {
            PORTD |= buzzer;
            _delay_ms(100);
            PORTD &= ~(buzzer);
            activate_actuator_ok = 0;
        }
        else if (activate_actuator_not_ok)
        {
            PORTD |= blueLED;
            _delay_ms(100);
            PORTD &= ~(blueLED);
            activate_actuator_not_ok = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
