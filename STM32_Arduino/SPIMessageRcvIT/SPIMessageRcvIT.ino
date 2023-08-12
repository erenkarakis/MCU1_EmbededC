
/*

 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *

 */

#include <SPI.h>

#define PendingMessagePin 9
#define MAX_LEN 255

bool bIsCompleteMessage = false;
uint8_t dataBuffer[MAX_LEN];
uint8_t cursor = 0;

/* Initalize SPI slave */
void SPI_SlaveInit(void)
{
  /* Initialize SPI pins */
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  /* Enable SPI as slave
   * SPCR-> SPI Configure Reg.
   * SPE-> SPI Enable
   */
  SPCR = (1 << SPE);
}

uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  /* When a serial transfer is complete, the SPIF Flag is set */
  while(!(SPSR & (1 << SPIF)));
  /* Return data register */
  return SPDR;
}

void SPI_SlaveTransmit(uint8_t data)
{
  /* Start transmission */
  SPDR = data;

  /* Wait for transmission complete */
  /* When a serial transfer is complete, the SPIF Flag is set */
  while(!(SPDR & (1 << SPIF)));
}

void NotifyMaster()
{
  digitalWrite(PendingMessagePin, HIGH);
  delayMicroseconds(50);
  digitalWrite(PendingMessagePin, LOW);
}

void setup() {
  Serial.begin(9600);

  pinMode(PendingMessagePin, OUTPUT);

  SPI_SlaveInit();
  Serial.println("Slave initalized");
}

void loop() {
  

  Serial.println("Type anything and press \"Send\"");
  while(!bIsCompleteMessage)
  {
    if (Serial.available())
    {
      /* Read a byte of incoming serial data */
      char readByte = (char)Serial.read();
      /* Write the data in to the buffer */
      dataBuffer[cursor++] = readByte;
      if (readByte == '\r' || cursor == MAX_LEN)
      {
        bIsCompleteMessage = true;
        dataBuffer[cursor - 1] = '\0'; /* Replace '\r' with '\0' */
      }
    }
  }

  NotifyMaster();

  Serial.println("Your message: ");
  Serial.println((char*)dataBuffer);

  /* Transmit all the data inside buffer over SPI */
  for (uint16_t i = 0; i < cursor; i++)
  {
    SPI_SlaveTransmit(dataBuffer[i]);
  }

  /* Reset cursor and message complete for next data transfer */
  cursor = 0;
  bIsCompleteMessage = false;

  Serial.println("Message sent!");

  while(digitalRead(SS));
  Serial.println("Master ends communication");

}
