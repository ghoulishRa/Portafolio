/*
 * main.c
 *
 * Created: 6/11/2023 7:35:18 PM
 *  Author: Israel Macías
 */ 


#define F_CPU 8000000UL       /* Define frequency here its 8MHz */
#include <xc.h>
#include <util/delay.h>

uint8_t h_ADC;
uint8_t l_ADC;

uint16_t completo;


uint8_t Sensibilidad = 0.1; //sensibilidad en V/A para nuestro sensor
uint8_t offset = 0.008 ; // Equivale a la amplitud del ruido

uint16_t ADC_read (void){
	
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1 << ADIF))== 0);
	ADCSRA |= (1 << ADIF);
	h_ADC = ADCH;
	l_ADC = ADCL;
	completo = (h_ADC << 8) + l_ADC;
	return completo;
}


uint16_t get_corriente(uint16_t valor_adc)
{
	float voltajeSensor;
	float corriente=0;
	uint16_t corr = 0;
	uint16_t ImaxB=0;
	uint16_t IminB=0;
	
	for(int i = 0; i < 1000; i++)//realizamos mediciones durante 0.5 segundos
	{
		voltajeSensor = ADC_read() * (5 / 1023);//lectura del sensor
		corriente=((voltajeSensor-3.2)/S); //Ecuacion  para obtener la corriente
		
		if(corriente < 0) corriente = 0;
		corr = corriente * 1000;
		
		if(corr>ImaxB)ImaxB=corr;
		if(corr<IminB)IminB=corr;
		
	}
	corr = ((ImaxB-IminB)/2)-offset;
	if(corr > 65000) corr = 0;
	return corr;
}



//cambiar offset y cambiar sensibilidad (y voltajde de referencia... probablemente)

int main(void)
{
	
	//salida por pueto b donde esta tx
	DDRD |= (1<<PIND1); //salida por pueto b donde esta tx
	
	
	//palabras de control adc
	ADMUX = 0b10000010;
	ADCSRA= 0b10000111;
	
	
	//palabras de control serial
	UCSRA = 0;
	UCSRB |= (1 << TXEN);   /* Turn on transmission and reception */
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit char size */
	UBRRL = 51; //BAUD_PRESCALE;            /* Load lower 8-bits of the baud rate */
	UBRRH = 0;//(BAUD_PRESCALE >> 8);
	
	while(1){
		envia_serial(ADC_read());
	}
}


void envia_serial(uint16_t valor){
	uint8_t parte_baja = (uint8_t)valor;
	uint16_t parte_alta = (uint8_t)(valor >> 8);
	UDR = parte_alta;
	while ( !( UCSRA & (1<<UDRE)));
	UDR = parte_baja;
	while ( !( UCSRA & (1<<UDRE)));
	_delay_ms(10);
}
