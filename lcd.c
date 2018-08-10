#include "sensorhub.h"
#include "drdstate.h"

extern struct lcd_ring_buffer LCDTransmitRingBuffer;

static unsigned char Reverse[256] =
{
  0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0, 0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
  0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8, 0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
  0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4, 0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
  0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec, 0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
  0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2, 0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
  0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea, 0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
  0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6, 0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
  0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee, 0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
  0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1, 0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
  0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9, 0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
  0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5, 0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
  0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed, 0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
  0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3, 0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
  0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb, 0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
  0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7, 0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
  0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef, 0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
};

int LCD_IsBusy(void)
{
  return Chip_GPIO_GetPinState( LPC_GPIO, 1, 11);
}

int LCD_Putc( struct lcd_ring_buffer *rPointer, unsigned short data )
{
  int RetValue = 0;
  int i, nextByte;

  Chip_TIMER_Disable( LPC_TIMER1 );

  if ( rPointer -> readIndex == rPointer -> writeIndex )
  {
    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0) & ~SPI_TXDATCTL_EOT) | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x40);

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDAT = 0x0A;

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(Reverse[(data & 0xff)]);

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_MSTIDLE) );

    LPC_SPI1->TXDATCTL = ((LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK) | SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0);

    rPointer -> buffer[rPointer -> writeIndex] = LCD_RS | Reverse[(data & 0xff)];

    rPointer -> writeIndex += 1;

    if ( rPointer -> writeIndex >= rPointer -> size )
    {
      rPointer -> writeIndex = 0;
    }

    RetValue = 1;

    Chip_GPIO_SetPinState( LPC_GPIO, 1,  8, true );

    Chip_GPIO_SetPinState( LPC_GPIO, 1,  10, false );

    Chip_TIMER_MatchEnableInt( LPC_TIMER1, 3 );

    Chip_TIMER_Enable( LPC_TIMER1 );
  }
  else
  {
    nextByte = rPointer -> writeIndex + 1;

    if ( nextByte >= rPointer -> size )
    {
      nextByte = 0;
    }

    if ( nextByte != rPointer -> readIndex )
    {
      __disable_irq();

      rPointer -> buffer[rPointer -> writeIndex] = LCD_RS | Reverse[data];

      rPointer -> writeIndex = nextByte;

      __enable_irq();

      RetValue = 1;
    }
  }

  Chip_TIMER_Enable( LPC_TIMER1 );

  return( RetValue );
}

int LCD_Putcc( struct lcd_ring_buffer *rPointer, unsigned short data )
{
  int RetValue = 0;
  int i, nextByte;

  Chip_TIMER_Disable( LPC_TIMER1 );

  if ( rPointer -> readIndex == rPointer -> writeIndex )
  {
    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0) & ~SPI_TXDATCTL_EOT) | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x40);

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDAT = 0x0A;

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(Reverse[(data & 0xff)]);

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_MSTIDLE) );

    LPC_SPI1->TXDATCTL = ((LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK) | SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0);

    rPointer -> buffer[rPointer -> writeIndex] = Reverse[(data & 0xff)];

    rPointer -> writeIndex += 1;

    if ( rPointer -> writeIndex >= rPointer -> size )
    {
      rPointer -> writeIndex = 0;
    }

    RetValue = 1;

    Chip_GPIO_SetPinState( LPC_GPIO, 1,  8, false );

    Chip_GPIO_SetPinState( LPC_GPIO, 1,  10, false );

    Chip_TIMER_MatchEnableInt( LPC_TIMER1, 3 );

    Chip_TIMER_Enable( LPC_TIMER1 );
  }
  else
  {
    nextByte = rPointer -> writeIndex + 1;

    if ( nextByte >= rPointer -> size )
    {
      nextByte = 0;
    }

    if ( nextByte != rPointer -> readIndex )
    {
      rPointer -> buffer[rPointer -> writeIndex] = Reverse[data];

      rPointer -> writeIndex = nextByte;

      RetValue = 1;
    }
  }

  Chip_TIMER_Enable( LPC_TIMER1 );

  return( RetValue );
}

int LCD_Puts( struct lcd_ring_buffer *rPointer, unsigned char *buffer, int count )
{
  int RetValue = 0;
  int i, nextByte;

  Chip_TIMER_Disable( LPC_TIMER1 );

  for( i = 0;i < count;i++ )
  {
    if ( rPointer -> readIndex == rPointer -> writeIndex )
    {
      while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
      LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0) & ~SPI_TXDATCTL_EOT) | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x40);

      while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
      LPC_SPI1->TXDAT = 0x0A;

      while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
      LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(Reverse[(buffer[0])]);

      while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_MSTIDLE) );

      LPC_SPI1->TXDATCTL = ((LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK) | SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0);

      rPointer -> buffer[rPointer -> writeIndex] = LCD_RS | Reverse[buffer[0]];

      rPointer -> writeIndex += 1;

      if ( rPointer -> writeIndex >= rPointer -> size )
      {
        rPointer -> writeIndex = 0;
      }

      RetValue = 1;

      Chip_GPIO_SetPinState( LPC_GPIO, 1,  8, true );

      Chip_GPIO_SetPinState( LPC_GPIO, 1,  10, false );

      Chip_TIMER_MatchEnableInt( LPC_TIMER1, 3 );

      Chip_TIMER_Enable( LPC_TIMER1 );
    }
    else
    {
      nextByte = rPointer -> writeIndex + 1;

      if ( nextByte >= rPointer -> size )
      {
        nextByte = 0;
      }

      if ( nextByte != rPointer -> readIndex )
      {
        rPointer -> buffer[rPointer -> writeIndex] = LCD_RS | Reverse[buffer[i]];

        rPointer -> writeIndex = nextByte;

        RetValue += 1;
      }
      else
      {
        break;
      }
    }
  }

  Chip_TIMER_Enable( LPC_TIMER1 );

  return( RetValue );
}

void LCD_Init( struct _drd_state_ *state )
{
    // LCD Init
            
    LCD_Putcc( &LCDTransmitRingBuffer, 0x38 );                 // 8-bits , 2 lines
    LCD_Putcc( &LCDTransmitRingBuffer, 0x38 );                 // 8-bits , 2 lines
    LCD_Putcc( &LCDTransmitRingBuffer, 0x38 );                 // 8-bits , 2 lines
    LCD_Putcc( &LCDTransmitRingBuffer, 0x38 );                 // 8-bits , 2 lines
    LCD_Putcc( &LCDTransmitRingBuffer, 0x08 );                 // disp=on  cursor=on
    LCD_Putcc( &LCDTransmitRingBuffer, 0x01 );                 // disp=on  cursor=on
    LCD_Putcc( &LCDTransmitRingBuffer, 0x06 );                 // disp=on  cursor=on
    LCD_Putcc( &LCDTransmitRingBuffer, 0x0c );                 // disp=on  cursor=on
    LCD_Putcc( &LCDTransmitRingBuffer, 0x80 );                 // disp=on  cursor=on

    state -> LCDPresent = 1;

    state -> LCD_Curx = 1;
    state -> LCD_Cury = 1;
}

void LCD_Clrscr( struct _drd_state_ *state )
{
  if ( state -> LCDPresent == 0 )
  {
    return;
  }

  LCD_Putcc( &LCDTransmitRingBuffer, 0x01 );

  state -> LCD_Curx = 1;
  state -> LCD_Cury = 1;
}

void LCD_Gotoxy( struct _drd_state_ *state, int x, int y)
{
  uint8_t val8;

  if ( state -> LCDPresent == 0 )
  {
    return;
  }

  val8 = (uint8_t) x;
  val8--;

  if (y == 2)
  {
    val8 += 64;
  }

  val8 |= 0x80;

  LCD_Putcc( &LCDTransmitRingBuffer, val8 );
  state -> LCD_Curx = x;
  state -> LCD_Cury = y;
}
