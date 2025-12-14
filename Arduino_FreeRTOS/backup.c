//Arduino uno with base shield extension using FreeRTOS
/*  FreeRTOS V202112.00 */

#include <avr/io.h>
#include "FreeRTOS-Kernel/include/FreeRTOS.h" /* RTOS kernel functions. */
#include "task.h" /* RTOS task related API prototypes. */
#include "semphr.h" /* Semaphore related API prototypes. */


#include <util/delay.h>
#include <avr/interrupt.h>
// #include <cstring>

#define Idle_Priority   (tskIDLE_PRIORITY)


//tasks handler defined after the main
//wait for an RFIFD card to be presented
static void WaitingTask( void *pvParameters );

//read the RFID card and send it so the master device

// static void RFIDTask( void *pvParameters );
// //blink an led every second to show the system is alive
// static void LEDTask( void *pvParameters );
// //open the door(led) for 5 seconds when a valid card is presented
// //and activate the ultrasound sensor
// static void DoorTask( void *pvParameters );
// //monitor the ultrasound sensor and close the door after 5 seconds, only 1 person allowed
// //in the doorway at a time
// static void UltrasoundTask( void *pvParameters );
// static void CounterTask( void *pvParameters );

//constant to ease the reading....
// const uint8_t doorLed = _BV(PD2);
// const uint8_t greenLed = _BV(PD4);
// const uint8_t ultrasoundSensor = _BV(PD6);
// // UNO D8 is PB0
// const uint8_t RFIDReader = _BV(PB0);
// // UNO D10 is PB2 (typically SPI SS)
// const uint8_t MasterDevice = _BV(PB2);

// Configuration
#define F_CPU 16000000UL                            // Fréquence d'horloge de 16MHz (Arduino Uno)
#define BAUD_RATE 9600
#define UBRR_VALUE ((F_CPU / 16 / BAUD_RATE) - 1)   // Calcul du UBRR pour l'UART matériel (9600 bauds)
#define BIT_PERIOD_US 104                           // Période d'un bit pour 9600 bauds ≈ 104.16 µs

// Définitions des broches SoftSerial
#define SOFTSERIAL_RX_PIN PIND6                     // Arduino D6 (Broche PD6)
#define SOFTSERIAL_TX_PIN PIND7                     // Arduino D7 (Broche PD7)

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
void uart_init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_tx(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
}

void uart_write_buffer(unsigned char *data, int len) { for (int i = 0; i < len; i++) uart_tx(data[i]); }

int uart_available() { return (UCSR0A & (1 << RXC0)); }

unsigned char uart_rx() { return UDR0; }

void softserial_init()
{
    DDRD &= ~(1 << SOFTSERIAL_RX_PIN);
    DDRD |= (1 << SOFTSERIAL_TX_PIN);
    PORTD |= (1 << SOFTSERIAL_TX_PIN);
}

int softserial_available() { return !(PIND & (1 << SOFTSERIAL_RX_PIN)); }

unsigned char softserial_read()
{
    unsigned char data = 0;
    while (PIND & (1 << SOFTSERIAL_RX_PIN));

    cli();
    
    _delay_us(BIT_PERIOD_US / 2);

    for (int i = 0; i < 8; i++)
    {
        _delay_us(BIT_PERIOD_US);
        if (PIND & (1 << SOFTSERIAL_RX_PIN))
            data |= (1 << i);
    }
    _delay_us(BIT_PERIOD_US);
    
    sei();

    return data;
}

void clearBufferArray() { for (int i = 0; i < BUFFER_SIZE; i++) buffer[i] = 0; }

void twi_init_slave(uint8_t address)
{
    // 1. Définir l'adresse de l'esclave (TWI Address Register)
    TWAR = (address << 1);

    // 2. Activer le TWI (TWEN)
    // Activer les interruptions TWI (TWIE)
    // Activer l'ACK (TWEA) pour répondre à l'adresse
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

void debug_print(const char *s)
{
    while (*s)
    {
        uart_tx(*s++);
    }
}

// Convertit un octet (0-255) en sa représentation hexadécimale (2 caractères) et l'envoie.
void uart_print_hex_byte(uint8_t b)
{
    char hex_digits[] = "0123456789ABCDEF";

    // Imprimer le quartet de poids fort (MSB)
    uart_tx(hex_digits[(b >> 4) & 0x0F]);

    // Imprimer le quartet de poids faible (LSB)
    uart_tx(hex_digits[b & 0x0F]);
}

// Imprime un tableau d'octets en hexadécimal, avec un préfixe et un retour à la ligne.
void uart_print_hex_buffer(const unsigned char *buffer, int len)
{
    debug_print("Rx_SS: "); // Préfixe pour identifier la source
    for (int i = 0; i < len; i++)
    {
        uart_print_hex_byte(buffer[i]);
        uart_tx(' '); // Espace entre les bytes pour la lisibilité
    }
    debug_print("(len=");
    // Pour afficher la longueur, une fonction itoa simple serait nécessaire.
    // Nous allons juste imprimer le buffer pour l'instant.
    uart_tx('\n');
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
        if ((twi_command == 0x03) && (card_detected == 1)) send_card_uid = 1;
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
            if (twi_data_index < UID_SIZE) TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA); 
            else TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // Sinon, NACK (pour le dernier byte)
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

            if (twi_data_index == UID_SIZE) TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // NACK final
            else TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA); // ACK prochain
        }
        else TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE); // Plus de données, mais le Maître veut quand même lire. NACK et attendre.
        break;

    case 0xC0: // Byte transmis, NACK reçu (Fin de la lecture par le Maître)
    case 0xA0: // STOP ou Repeated START
    default:
        // Rendre le contrôle du bus, attendre le prochain START
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
        break;
    }
}

int main(void){
    // DDRD |= (doorLed | greenLed); // PD2 and PD4 as outputs
    // DDRD &= ~ultrasoundSensor;    // PD6 as input
    // DDRB &= ~(RFIDReader | MasterDevice); // PB0 (D8) and PB2 (D10) as inputs

    uart_init(UBRR_VALUE); // UART Matériel (PC)
    softserial_init();
    twi_init_slave(TWI_SLAVE_ADDRESS); // TWI Slave (I2C)
    sei();

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
    while (1)
    {
        // ======tache 1======
        // reception infos carte
        if (softserial_available())
        {
            // Lecture des données SoftSerial (l'UID de la carte)
            while (softserial_available() && (count < BUFFER_SIZE))
                buffer[count++] = softserial_read();
            if (count > 0)
            {
                uart_print_hex_buffer(buffer, count);
                card_detected = 1;
                int uid_len = (count < UID_SIZE) ? count : UID_SIZE;
                // memcpy(current_uid, buffer, uid_len);
                // Copie manuelle de l'UID (équivalent à memcpy(current_uid, buffer, uid_len))
                for (int i = 0; i < uid_len; i++)
                    current_uid[i] = buffer[i];
                // S'assurer que la chaîne de caractères est terminée par un NULL (si le format est ASCII)
                if (uid_len < UID_SIZE)
                    current_uid[uid_len] = 0;
                PORTD ^= LED_PIN;
                clearBufferArray();
                count = 0;
            }
        }
        // ======tache 1======
    }
    // for(;;){
    //     // Simple heartbeat: toggle green LED every 500ms
    //     PORTD ^= greenLed;
    //     vTaskDelay(pdMS_TO_TICKS(500));
    // }
}
