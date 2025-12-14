// Code C pour ATmega328P (AVR-C) - Implémentation Hardware/Software UART Bridge
// Modifié pour allumer une LED (D13) lorsque des données sont reçues du SoftSerial.

#include <avr/io.h>
#include <util/delay.h>

// --- Configuration ---
#define F_CPU 16000000UL // Fréquence d'horloge de 16MHz (Arduino Uno)
#define BAUD_RATE 9600

// Calcul du UBRR pour l'UART matériel (9600 bauds)
#define UBRR_VALUE ((F_CPU / 16 / BAUD_RATE) - 1)

// Définitions des broches SoftSerial
#define SOFTSERIAL_RX_PIN PIND2 // Arduino D2 (Broche PD2)
#define SOFTSERIAL_TX_PIN PIND3 // Arduino D3 (Broche PD3)

// Définitions de la LED (Ajouté)
#define LED_PIN PB5 // Broche D13 de l'Arduino Uno (Port B, bit 5)

// Période d'un bit pour 9600 bauds ≈ 104.16 µs
#define BIT_PERIOD_US 104

// Buffer
#define BUFFER_SIZE 64
unsigned char buffer[BUFFER_SIZE];
int count = 0;

// --- 1. Fonctions d'UART Matériel (Liaison PC) ---
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

void uart_write_buffer(unsigned char *data, int len)
{
  for (int i = 0; i < len; i++)
  {
    uart_tx(data[i]);
  }
}

// int uart_available()
// {
//   return (UCSR0A & (1 << RXC0));
// }

unsigned char uart_rx()
{
  return UDR0;
}

// --- 2. Fonctions de SoftSerial Logiciel (Bit-Banging) ---

// void softserial_init()
// {
//   DDRD &= ~(1 << SOFTSERIAL_RX_PIN);
//   DDRD |= (1 << SOFTSERIAL_TX_PIN);
//   PORTD |= (1 << SOFTSERIAL_TX_PIN);
// }

// void softserial_write(unsigned char data)
// {
//   PORTD &= ~(1 << SOFTSERIAL_TX_PIN);
//   _delay_us(BIT_PERIOD_US);

//   for (int i = 0; i < 8; i++)
//   {
//     if (data & 0x01)
//       PORTD |= (1 << SOFTSERIAL_TX_PIN);
//     else
//       PORTD &= ~(1 << SOFTSERIAL_TX_PIN);

//     _delay_us(BIT_PERIOD_US);
//     data >>= 1;
//   }

//   PORTD |= (1 << SOFTSERIAL_TX_PIN);
//   _delay_us(BIT_PERIOD_US);
// }

// int softserial_available()
// {
//   return !(PIND & (1 << SOFTSERIAL_RX_PIN));
// }

// unsigned char softserial_read()
// {
//   unsigned char data = 0;
//   while (PIND & (1 << SOFTSERIAL_RX_PIN))
//     ;
//   _delay_us(BIT_PERIOD_US / 2);

//   for (int i = 0; i < 8; i++)
//   {
//     _delay_us(BIT_PERIOD_US);
//     if (PIND & (1 << SOFTSERIAL_RX_PIN))
//     {
//       data |= (1 << i);
//     }
//   }
//   _delay_us(BIT_PERIOD_US);
//   return data;
// }

// --- 3. Fonctions de Support ---

// Initialisation de l'UART (9600, 8N1)
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

// Transmission d'un seul caractère
void uart_transmit(unsigned char data)
{
    // Attendre que le tampon de transmission soit vide
    while (!(UCSR0A & (1 << UDRE0)))
        ;

    // Placer les données dans le tampon et les envoyer
    UDR0 = data;
}

// Réception d'un seul caractère
unsigned char uart_receive(void)
{
    // Attendre que les données soient reçues
    while (!(UCSR0A & (1 << RXC0)));

    // Obtenir et retourner les données du tampon
    return UDR0;
}

// Vérifier si des données sont disponibles en réception
int uart_available(void)
{
    return (UCSR0A & (1 << RXC0));
}

// --- Fonctions de Tampon (Buffer) ---

// Fonction pour effacer le tableau de tampons
void clearBufferArray(int size)
{
    for (int i = 0; i < size; i++)
    {
        buffer[i] = 0; // Utilisation de 0 (NULL pour char) au lieu de NULL macro
    }
}

void clearBufferArray()
{
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    buffer[i] = 0;
  }
}

// --- 4. Fonction Principale (main) ---

int main(void)
{
  // Initialisation des ports série
  uart_init(); // UART Matériel (PC)
  //softserial_init();     // SoftSerial (Shield)

  // NOUVEAU : Initialisation de la LED (PB5/D13 comme sortie)
  DDRB |= (1 << LED_PIN);
  // Éteindre la LED initialement (mettre PB5 à LOW)
  PORTB &= ~(1 << LED_PIN);

  // Boucle principale
  while (1)
  {
    // // === Flux 1 : SoftSerial Shield vers Action (LED) ===
    // if (softserial_available())
    // {
    //   // Lecture des données SoftSerial (l'UID de la carte)
    //   while (softserial_available() && (count < BUFFER_SIZE))
    //   {
    //     buffer[count++] = softserial_read();
    //   }

    //   if (count > 0)
    //   {
    //     // L'UID a été lu (simulé par la réception des données)

    //     // NOUVEAU : Allumer la LED (remplace ou complète l'envoi au PC)
    //     PORTB |= (1 << LED_PIN); // Mettre la broche LED à HIGH

    //     _delay_ms(500); // Garder la LED allumée pendant 500ms

    //     PORTB &= ~(1 << LED_PIN); // Éteindre la LED (Mettre la broche LED à LOW)

    //     // --- FIN de l'action ---

    //     // Réinitialiser le buffer pour la prochaine lecture
    //     clearBufferArray();
    //     count = 0;
    //   }
    // }

    // // === Flux 2 : PC vers SoftSerial Shield (Identique) ===
    // if (uart_available())
    // {
    //   unsigned char data = uart_rx();
    //   softserial_write(data);
    // }
    if (uart_available())
    { // Si des données arrivent sur le port série
      count = 0;
      // Lecture des données dans le tableau (lecture en rafale)
      while (uart_available() && count < BUFFER_SIZE)
      {
        buffer[count++] = uart_receive();
      }

      // Écriture du tampon (envoi en rafale au PC)
      for (int i = 0; i < count; i++)
      {
        uart_transmit(buffer[i]);
      }

      // Nettoyage et réinitialisation
      clearBufferArray(count);
      count = 0;
    }
  }

  return 0;
}