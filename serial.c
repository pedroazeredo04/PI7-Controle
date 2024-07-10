/*

Interface PIC

Designed by Shane Tolmie Feb 2001.

Microprocessor: Microchip PIC16F87x

Compiled with:	Hitech-C v7.87, developed using MPLAB v5.3

Note:						all references are to PIC16C7X PDF version of Microchip manual, DS30390E

Overall goal: 	serial comms using USART to comm port of an ibm pc compatible computer

Modified by Jun Okamoto Jr. for the purpose of the PMR2415 course,
in December 2009. Follow changes by the tag [jo:yymmdd].

Atualizado por Jun Okamoto Jr. para PI-7 em 25/10/2023
usando 115200 baud

*/

#include <xc.h>
#include	<conio.h>
#include	<stdio.h>
#include	"always.h"
#include	"delay.h"         // PIC_CLK definido aqui

void ser_init(void)
{
	/* relates crystal freq to baud rate - see above and PIC16F87x data sheet under 'USART async. modes'

	BRGH=1, Fosc=3.6864MHz		BRGH=1, Fosc=4MHz      BRGH=1, Fosc=8MHz    BRGH=1, Fosc=16MHz
	----------------------	  -----------------      -----------------    ------------------
	Baud		SPBRG             Baud		SPBRG          Baud		SPBRG         Baud		SPBRG
	1200		191               1200		207.3          1200		415.7         9600    103
	2400		95                2400		103.2          2400		207.3         19200		51
	4800		47                4800		51.1           4800		103.2         38400   25
	9600		23                9600		25.0           9600		51.1          57600   16
	19200		11                19200		12.0           19200	25.0          115200  8
	38400		5                 38400		5.5            38400	12.0
	57600		3                 57600		3.3            57600	7.7
	115200	1                 115200	1.2            115200	3.3

	*/

	/*
	 * Comms setup:
	 */

	//#define BAUD 19200 // [jo:231025] original
  #define BAUD 115200  // [jo:231025] para PI-7 2024 � 115200 baud
	//#define DIVIDER ((PIC_CLK/(16UL * BAUD) -1)) // [jo:231025] para SYNC=0, BRGH=0 e BRG16=1
	#define DIVIDER ((PIC_CLK/(4UL * BAUD) -1)) // [jo:231025] para SYNC=0, BRGH=1 e BRG16=1
	#define HIGH_SPEED 1

	//you can comment these #assert statements out if you dont want error checking
	#if PIC_CLK==3686400 && BAUD==19200
		#assert DIVIDER==11
	#elif PIC_CLK==4000000 && BAUD==19200
		#assert DIVIDER==12
	#elif PIC_CLK==16000000 && BAUD==19200
		#assert DIVIDER==51
	#elif PIC_CLK==20000000 && BAUD==19200
		#assert DIVIDER==64
	#endif

	SPBRG=DIVIDER;
	BRGH=HIGH_SPEED;	//data rate for sending
  BRG16 = 1;        // [jo:231025]
  SPBRGH = 0;       // [jo:231025]
	SYNC=0;						//asynchronous
	SPEN=1;						//enable serial port pins
	CREN=1;						//enable reception
	SREN=0;						//no effect
	TXIE=0;						//disable tx interrupts
	RCIE=0;						//disable rx interrupts
	TX9=0;						//8-bit transmission
	RX9=0;						//8-bit reception
	TXEN=0;						//reset transmitter
	TXEN=1;						//enable the transmitter
}

unsigned char dummy;

#define clear_usart_errors_inline	\
if (OERR)													\
{																	\
	TXEN=0;													\
	TXEN=1;													\
	CREN=0;													\
	CREN=1;													\
}																	\
if (FERR)													\
{																	\
	dummy=RCREG;										\
	TXEN=0;													\
	TXEN=1;													\
}

//writes a character to the serial port
void putch(char c)
{
	while(!TXIF)			//set when register is empty
	{
		clear_usart_errors_inline;
		CLRWDT();
	}
	TXREG=c;
//	DelayUs(60);
}

//gets a character from the serial port without timeout
char getch(void)
{
	while(!RCIF)
	{
		CLRWDT();
		clear_usart_errors_inline;
	}
	return RCREG;
}

unsigned char getch_timeout(void)
{
	unsigned char i;
	long timeout_int;

	// retrieve one byte with a timeout
	for (i=2;i!=0;i--)
	{
		timeout_int=timeout_int_us(240000);

		while (hibyte(timeout_int)!=0) //only check the msb of the int for being 0, it saves space, see always.h for macro
		{
			CLRWDT();
			timeout_int--;
			if (RCIF)
			{
				return RCREG;
			}
		}
	}
	return 0;
}

// [jo:091211] Verifica se um caracter chegou pela serial. Se chegou retorna o caracter 
//             sen�o retorna 255
unsigned char chkchr (void) {
  if (RCIF) {      // se recebeu algo
    return RCREG;  // retorna o caracter recebido
  }
  return 255;      // sen�o retorna 255
}

void clear_usart_errors(void)
{
	clear_usart_errors_inline;
}



/*
writes a character to the serial port in hex
if serial lines are disconnected, there are no errors
*/

void putchhex(unsigned char c)
{
	unsigned char temp;

	// transmits in hex

	temp=c;

	c=(c >> 4);
	if (c<10) c+=48; else c+=55;
	putch(c);

	c=temp;

	c=(c & 0x0F);
	if (c<10) c+=48; else c+=55;
	putch(c);
}

void putinthex(unsigned int c)
{
	#define ramuint(x)						(*((unsigned int *) (x)))
	#define ramuint_hibyte(x)			(*(((unsigned char *)&x)+1))
	#define ramuint_lobyte(x)			(*(((unsigned char *)&x)+0))
	#define ramuchar(x)						(*((unsigned char *) (x)))

	putchhex(ramuint_hibyte(c));
	putchhex(ramuint_lobyte(c));

	#undef ramuint
	#undef ramuint_hibyte
	#undef ramuint_lobyte
	#undef ramuchar
}

//if there has been a previous timeout error from getch_timeout, this returns TRUE
unsigned char usart_timeout(void)
{
//	return usart_timeout_error;
	return FALSE;
}

/*
writes a character to the serial port in decimal
if serial lines are disconnected, there are no errors
*/
void putchdec(unsigned char c)
{
	unsigned char temp;

	temp=c;
	//hundreds
	if ((c/100)>0) putch((c/100)+'0');
	c-=(c/100)*100;

	//tens
	if (((temp/10)>0) || ((temp/100)>0)) putch((c/10)+'0');
	c-=(c/10)*10;

	//ones
	putch((c/1)+'0');
}

void putst(register const char *str)
{
	while((*str)!=0)
	{
		putch(*str);
    if (*str==13) putch(10);
    if (*str==10) putch(13);
		str++;
	}
}

