// Arduino uno with base shield extension using FreeRTOS
/*  FreeRTOS V202112.00 */

#define BAUD 9600
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

// Configuration
// #define F_CPU 16000000UL // Fréquence d'horloge de 16MHz (Arduino Uno)
// #define BAUD_RATE 9600
// #define UBRR_VALUE ((F_CPU / 16 / BAUD_RATE) - 1) // Calcul du UBRR pour l'UART matériel (9600 bauds)
// #define BIT_PERIOD_US 104                         // Période d'un bit pour 9600 bauds ≈ 104.16 µs

// Buffer pour l'UART
#define UID_SIZE 16
unsigned char buffer[UID_SIZE];
int count = 0;

// // Définitions de la LED (Ajouté)
// #define LED_PIN _BV(PD4) // Broche D13 de l'Arduino Uno (Port B, bit 5)
#define IR_EMITTER_A _BV(PD6)
#define blueLED _BV(PD5)
// Config I2C
#define SLAVE_ADRESS 0x42 // Adresse I2C de l'Arduino

// Variables globales
unsigned char current_uid[UID_SIZE] = {0};
; // Initialisé à zéro
volatile uint8_t card_detected = 0;
volatile uint8_t State_IR_A = 0;
volatile uint8_t nb_passage = 0;
volatile uint8_t activate_actuator = 0;

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

// =================================================
// --- 1. Fonctions detection sens de passage IR ---
// =================================================
extern "C"
{
    ISR(INT0_vect)
    {
        State_IR_A = 1;
    }
}

// =================================================
// --- 2. Fonctions d'UART Matériel (Liaison PC) ---
// =================================================

// Instanciation globale pour accès dans l'ISR
RfidTask rfidReader;

// --- Interruption de Réception UART ---
ISR(USART_RX_vect)
{
    uint8_t receivedByte = UDR0; // Lire le registre de données UART
    rfidReader.processIncomingByte(receivedByte);
}

int main(void)
{

    PORTD = IR_EMITTER_A;

    DDRD = 0b11111011;
    EICRA = 0b00000010;
    EIMSK |= (1 << INT0);

    // uart_init(); // UART Matériel (PC)
    rfidReader.init();
    Wire.begin(SLAVE_ADRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    sei();

    // Create tasks
    xTaskCreate(RFIDReadTask, "RFIDReadTask", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(ActivateActuatorTask, "ActivateActuatorTask", 128, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    return 0;
}

void receiveEvent(int howMany)
{
    if (!Wire.available()) return;

    char command = Wire.read();

    if (command == 'F') activate_actuator = 1;
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
            uint8_t len = rfidReader.getUidLength();

            for (int i=0; i<len; i++) current_uid[i] = uid[i];
        }
    }
}

static void ActivateActuatorTask(void *pvParameters)
{
    while(1)
    {
        if (activate_actuator)
        {
            PORTD |= blueLED;
            _delay_ms(100);
            PORTD &= ~(blueLED);
            activate_actuator = 0;
        }
    }
}

// Minimal implementation so the project links
// static void RFIDReadTask(void *pvParameters)
// {
//     while (1)
//     {
//         // ======tache 1======
//         // reception infos carte
//         if (uart_available())
//         { // Si des données arrivent sur le port série
//             count = 0;
//             taskENTER_CRITICAL();
//             // Lecture des données dans le tableau (lecture en rafale)
//             while (uart_available() && count < UID_SIZE) { buffer[count++] = uart_receive(); }

//             // Écriture du tampon (envoi en rafale au PC)
//             for (int i = 0; i < count; i++) current_uid[i] = buffer[i];

//             // Nettoyage et réinitialisation
//             clearBufferArray();

//             for (int i = 0; i < count; i++) uart_transmit(current_uid[i]);
//             card_detected = 1;
//             count = 0;
//             taskEXIT_CRITICAL();
//         }
//         // ======tache 1======
//     }
// }

// static void TraversalDirectionTask(void *pvParameters)
// {
//     (void)pvParameters;
//     uint8_t first = 0; // 1 si A, 2 si B
//     // A -> B, un utilisateur rentre
//     // B -> A, un utilisateur sort
//     while (1)
//     {
//         if (State_IR_A)
//         {
//             PORTD ^= blueLED;
//             cli();
//             nb_passage++;
//             uart_transmit((unsigned char)nb_passage);
//             State_IR_A = 0;
//             sei();
//         }
//         // if (State_IR_A && State_IR_B)
//         // {
//         //     if (first == 1) in += 1;
//         //     else if (first == 2) out += 1;
//         //     State_IR_A = 0;
//         //     State_IR_B = 0;
//         // }
//         // else if (State_IR_A) first = 1;
//         // else if (State_IR_B) first = 2;
//         // uart_transmit((unsigned char) State_IR_A);
//         // uart_transmit((unsigned char) State_IR_B);
//     }
// }