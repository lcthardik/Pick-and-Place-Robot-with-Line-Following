
void buzzer_pin_config (void)
{
 DDRB = DDRB | 0x02;		//Setting PORTB 1 as output
 PORTB = PORTB & 0xFD;		//Setting PORTB 1 logic low to turnoff buzzer
}

void port_init_buzzer (void)
{
 buzzer_pin_config();
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINB;
 port_restore = port_restore | 0x02;
 PORTB = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINB;
 port_restore = port_restore & 0xFD;
 PORTB = port_restore;
}

void init_devices_buzzer (void)
{
 cli(); //Clears the global interrupts
 port_init_buzzer();
 sei(); //Enables the global interrupts
}



void sound_on()
{
  buzzer_on();
    _delay_ms(7000);    //delay
    buzzer_off();
    _delay_ms(1000);  


}

