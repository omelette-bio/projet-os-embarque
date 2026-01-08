// Arduino uno with base shield extension using FreeRTOS
/*  FreeRTOS V202112.00 */

#include <avr/io.h>
#include "FreeRTOS-Kernel/include/FreeRTOS.h" /* RTOS kernel functions. */
#include "task.h"                             /* RTOS task related API prototypes. */
#include "semphr.h"                           /* Semaphore related API prototypes. */

#include <util/delay.h>
#include <avr/interrupt.h>
// #include <cstring>

#define Idle_Priority (tskIDLE_PRIORITY)

static void WaitingTask(void *pvParameters);

// Configuration
#define F_CPU 16000000UL // Fréquence d'horloge de 16MHz (Arduino Uno)
#define BAUD_RATE 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD_RATE) - 1) // Calcul du UBRR pour l'UART matériel (9600 bauds)
#define BIT_PERIOD_US 104                         // Période d'un bit pour 9600 bauds ≈ 104.16 µs

// Définitions des broches SoftSerial
#define SOFTSERIAL_RX_PIN PIND6 // Arduino D6 (Broche PD6)
#define SOFTSERIAL_TX_PIN PIND7 // Arduino D7 (Broche PD7)

// Buffer pour le SoftSerial
#define BUFFER_SIZE 64
unsigned char buffer[BUFFER_SIZE];
int count = 0;

// Définitions de la LED (Ajouté)
#define LED_PIN _BV(PD4) // Broche D13 de l'Arduino Uno (Port B, bit 5)

// Config I2C
#define TWI_SLAVE_ADDRESS 0x42 // Adresse I2C de l'Arduino

// Variables globales
#define UID_SIZE 16
unsigned char current_uid[UID_SIZE] = {0};
; // Initialisé à zéro
int card_detected = 0;
int send_card_uid = 1;

volatile uint8_t twi_command = 0;
volatile uint8_t twi_data_index = 0;

// =================================================
// --- 1. Fonctions d'UART Matériel (Liaison PC) ---
// =================================================
void uart_init(void)
{
    // Définir le taux de baud
    UBRR0H = (unsigned char)(UBRR_VALUE >> 8);
    UBRR0L = (unsigned char)UBRR_VALUE;

    // Activer l'émetteur et le récepteur
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Définir le format de trame : 8 bits de données, pas de parité, 1 bit de stop
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(unsigned char data)
{
    // Attendre que le tampon de transmission soit vide
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    // Placer les données dans le tampon et les envoyer
    UDR0 = data;
}

unsigned char uart_receive(void)
{
    // Attendre que les données soient reçues
    while (!(UCSR0A & (1 << RXC0)))
        ;

    // Obtenir et retourner les données du tampon
    return UDR0;
}

// Vérifier si des données sont disponibles en réception
int uart_available(void) { return (UCSR0A & (1 << RXC0)); }

void clearBufferArray()
{
    for (int i = 0; i < BUFFER_SIZE; i++)
        buffer[i] = 0;
}

void twi_init_slave(uint8_t address)
{
    // 1. Définir l'adresse de l'esclave (TWI Address Register)
    TWAR = (address << 1);

    // 2. Activer le TWI (TWEN)
    // Activer les interruptions TWI (TWIE)
    // Activer l'ACK (TWEA) pour répondre à l'adresse
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

ISR(TWI_vect)
{
    uint8_t status = TWSR & 0xF8; // Masquer les bits de prescaler

    switch (status)
    {

    // --- MASTER WRITE (RÉCEPTION DE COMMANDE) ---
    case 0x60:           // SLA+W reçu, ACK envoyé
    case 0x68:           // Arbitration perdu mais SLA+W reçu, ACK envoyé
        twi_command = 0; // Réinitialiser la commande
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
        // TWCR configuré par défaut à la fin du switch (TWEN | TWIE | TWEA)
        break;

    case 0x80:
        twi_command = TWDR;
        if ((twi_command == 0x03) && (card_detected == 1))
            send_card_uid = 1;
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
        break;

    // --- MASTER READ (TRANSMISSION DE L'UID) ---
    case 0xA8: // SLA+R reçu, ACK envoyé
    case 0xB0: // Arbitration perdu mais SLA+R reçu, ACK envoyé
        if (send_card_uid == 1)
        {
            // La commande 0x03 a été reçue. Préparer l'envoi de l'UID.
            twi_data_index = 0;
            TWDR = current_uid[twi_data_index]; // Envoyer le premier byte
            twi_data_index++;

            // Si ce n'est pas le dernier byte, on garde TWEA (ACK) actif
            if (twi_data_index < UID_SIZE)
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
            else
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // Sinon, NACK (pour le dernier byte)
        }
        else
        {
            // Commande non reconnue ou aucune commande. Envoyer une valeur par défaut.
            TWDR = 0x1A;
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // NACK (fin de transmission)
        }
        break;

    case 0xB8: // Byte de donnée transmis, ACK reçu (le Maître veut le suivant)
        if (twi_data_index < UID_SIZE)
        {
            TWDR = current_uid[twi_data_index]; // Envoyer le byte suivant
            twi_data_index++;

            if (twi_data_index == UID_SIZE)
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // NACK final
            else
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA); // ACK prochain
        }
        else
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // Plus de données, mais le Maître veut quand même lire. NACK et attendre.
        break;

    case 0xC0: // Byte transmis, NACK reçu (Fin de la lecture par le Maître)
    case 0xA0: // STOP ou Repeated START
    default:
        // Rendre le contrôle du bus, attendre le prochain START
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
        break;
    }
}

int main(void)
{
    // DDRD |= (doorLed | greenLed); // PD2 and PD4 as outputs
    // DDRD &= ~ultrasoundSensor;    // PD6 as input
    // DDRB &= ~(RFIDReader | MasterDevice); // PB0 (D8) and PB2 (D10) as inputs

    uart_init(); // UART Matériel (PC)
    // softserial_init();
    twi_init_slave(TWI_SLAVE_ADDRESS); // TWI Slave (I2C)
    sei();
    PORTD = 0;

    // Create tasks
    xTaskCreate(
        WaitingTask,
        "WaitingTask",
        128,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

    // Start scheduler.
    vTaskStartScheduler();

    return 0;
}

// Minimal implementation so the project links
static void WaitingTask(void *pvParameters)
{
    (void)pvParameters;
    while (1)
    {
        // ======tache 1======
        // reception infos carte
        if (uart_available())
        { // Si des données arrivent sur le port série
            count = 0;
            // Lecture des données dans le tableau (lecture en rafale)
            while (uart_available() && count < BUFFER_SIZE) { buffer[count++] = uart_receive(); }

            // Écriture du tampon (envoi en rafale au PC)
            for (int i = 0; i < count; i++)
            {
                current_uid[i] = buffer[i];
                uart_transmit(buffer[i]);
            }

            // Nettoyage et réinitialisation
            clearBufferArray();
            count = 0;
        }
        // ======tache 1======
    }
}
