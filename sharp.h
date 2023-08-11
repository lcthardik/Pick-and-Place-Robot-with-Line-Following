

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance;
unsigned int value;




// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 



// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}




//Main Function
/*int main(void)
{
	unsigned int value;
	init_devices();
	

	
	while(1)
	{						//Analog Value Of Front Sharp Sensor

		sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
		lcd_print(2,14,value,3); 						//Prints Value Of Distanc in MM measured by Sharp Sensor.
	}
}*/

int sharp_sensor_distance()
{
	unsigned int value;
	sharp = ADC_Conversion(9);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value = Sharp_GP2D12_estimation(sharp);
	return value;
}

