#include "eqep.h"
#include <ctime>
#include <chrono>
#include "PWM.h"
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "i2c.h"

#define DEVID		0x00
#define CTRL1		0x20
#define CTRL2	 	0x21
#define CTRL5	 	0x24
#define CTRL6	 	0x25
#define CTRL7	 	0x26
#define OUT_X_L_G	0x28
#define OUT_X_H_G	0x29
#define OUT_Y_L_G	0x2a
#define OUT_Y_H_G	0x2b
#define OUT_Z_L_G	0x2c
#define OUT_Z_H_G	0x2d
#define OUT_X_L_A	0x28
#define OUT_X_H_A	0x29
#define OUT_Y_L_A	0x2a
#define OUT_Y_H_A	0x2b
#define OUT_Z_L_A	0x2c
#define OUT_Z_H_A	0x2d
#define OUT_X_L_M	0x08
#define OUT_X_H_M	0x09
#define OUT_Y_L_M	0x0a
#define OUT_Y_H_M	0x0b
#define OUT_Z_L_M	0x0c
#define OUT_Z_H_M	0x0d
#define GSENSOR		0x6b
#define AMSENSOR	0x1d


using namespace exploringBB;

int main (int argc, char** argv)
{

	int file;
	int data;
	using namespace std::chrono;
	steady_clock::time_point t1;
	steady_clock::time_point t2;
	duration<double> time_span;
	double dt,avt=0;
	uint32_t x1, x2, dx;
	float vel=0;
	char dir[]= "/sys/devices/ocp.3/48300000.epwmss/48300180.eqep"; 
    	float target = atof(argv[1]), feedback;
	float acc[3]; //0 -x , 1 - y, 2 - z
	int gyr[3]; //0 -x , 1 - y, 2 - z
	int mag[3]; //0 -x , 1 - y, 2 - z

    // Allocate instance of pwm
    PWM pwm("pwm_test_P9_22.15");  // P9_22 MUST be loaded as a slot before use
    PWM pwm2("pwm_test_P9_21.16"); // P9_21 MUST be loaded as a slot before use
    pwm.setPeriod(100000);         // Set the period in ns
    pwm2.setPeriod(100000);        // Set the period in ns
    pwm.setDutyCycle(25.0f);       // Set the duty cycle as a percentage
    pwm.setPolarity(PWM::ACTIVE_LOW);  // using active low PWM
    pwm2.setDutyCycle(100.0f - target);  // Set the duty cycle as a percentage
    pwm2.setPolarity(PWM::ACTIVE_LOW);  // using active low PWM
    pwm.run();                     // start the PWM output
    pwm2.run();                     // start the PWM output
    // Allocate an instance of eqep
    eQEP eqep1(dir, eQEP::eQEP_Mode_Absolute);
    // Set the unit time period to 100,000,000 ns, or 0.1 seconds
    eqep1.set_period(100000000L);
    // Query back the period
    std::cout << "eQEP " << argv[1] << "] Period = " << eqep1.get_period() << " ns" << std::endl;
/////////////////////////////////////////////////////////////////////////////////////////////////   
	printf("Starting the LSM303D test application\n");
	if(  ( file=open("/dev/i2c-1",O_RDWR) ) < 0  ){
		perror("Failed to open i2c bus\n");
		return 1;}
	if(argv[2][0]=='g'){
		if( ioctl(file,I2C_SLAVE,GSENSOR )<0 ){
			perror("Failed to connect to the sensor\n");
			return 1;}

		data = i2c_smbus_read_byte_data(file,DEVID);
		if (data < 0){
			perror("Failed to read data at DEVID \n");
			return 1;}
		else {
			printf("DEVID is 0x%02x \n",data);}	
	
		data = i2c_smbus_read_byte_data(file,CTRL1);
		printf("Value of register 0x%02x before write action is: 0x%02x\n",CTRL1,data);		
	
		i2c_smbus_write_byte_data(file,CTRL1,0x0f);
		data = i2c_smbus_read_byte_data(file,CTRL1);
	
		printf("Value of register 0x%02x after write action is: 0x%02x\n",CTRL1,data);
	
		//writing  all other registers
		i2c_smbus_write_byte_data(file,CTRL2,0x20);	
		//Reading values of magnetometer
	}
	else if (argv[2][0]=='a'){
		 if( ioctl(file,I2C_SLAVE,AMSENSOR )<0 ){
                        perror("Failed to connect to the sensor\n");
                        return 1;}

                data = i2c_smbus_read_byte_data(file,DEVID);
                if (data < 0){
                        perror("Failed to read data at DEVID \n");
                        return 1;}
                else {
                        printf("DEVID is 0x%02x \n",data);}     
        
                data = i2c_smbus_read_byte_data(file,CTRL1);
		printf("Value of register 0x%02x before write action is: 0x%02x\n",CTRL1,data);		

        
                i2c_smbus_write_byte_data(file,CTRL1,0x57);
                data = i2c_smbus_read_byte_data(file,CTRL1);

		printf("Value of register 0x%02x after write action is: 0x%02x\n",CTRL1,data);


                //writing  all other registers
                i2c_smbus_write_byte_data(file,CTRL2,0x18);     
                i2c_smbus_write_byte_data(file,CTRL5,0x64);     
                i2c_smbus_write_byte_data(file,CTRL6,0x20);
                i2c_smbus_write_byte_data(file,CTRL7,0x00);     
     
                //Reading values of magnetometer
	}
/////////////////////////////////////////////////////////////////////////////////////
    // Read position indefintely
    while(1)
    {
	t1 = steady_clock::now();x1 = eqep1.get_position();
	t2 = steady_clock::now();x2 = eqep1.get_position();
	time_span = duration_cast<duration<double>>(t2-t1);
	dt = time_span.count();
	if(x2>x1) dx = x2-x1;
	else dx = x1-x2;
	avt = (avt+dt)/2;
	vel=(float)(dx/dt);
	feedback = (float)(vel*100.0/30000.0);
	pwm.setDutyCycle(100.0f - feedback*1.0f);
//	if(argv[2][0]=='g'){
//		gyr[0] = (i2c_smbus_read_byte_data(file,OUT_X_H_G)<<8) | i2c_smbus_read_byte_data(file,OUT_X_L_G);
//		gyr[1] = (i2c_smbus_read_byte_data(file,OUT_Y_H_G)<<8) | i2c_smbus_read_byte_data(file,OUT_Y_L_G);
//		gyr[2] = (i2c_smbus_read_byte_data(file,OUT_Z_H_G)<<8) | i2c_smbus_read_byte_data(file,OUT_Z_L_G);
//	
//		for (int j =0 ; j<3; j++)
//		{
//		    if(gyr[j] >= 32768)
//		    {
//		         gyr[j] =  32768*2 - gyr[j];
//		         gyr[j] = -gyr[j];
//		    }
//		}
//		/*
//		x = (gyr[0]*180/32768);
//		y = (gyr[1]*180/32768);
//		z = (gyr[2]*180/32768);
//		*/
//		std::cout << std::fixed  << "Gyr "  << " X: " << gyr[0]*180/32768 << " Y: " << gyr[1] << " Z: " << gyr[2] << std::endl;
//	}
        if(argv[2][0]=='a'){
		acc[0] = (i2c_smbus_read_byte_data(file,OUT_X_H_A)<<8) | i2c_smbus_read_byte_data(file,OUT_X_L_A);
		acc[1] = (i2c_smbus_read_byte_data(file,OUT_Y_H_A)<<8) | i2c_smbus_read_byte_data(file,OUT_Y_L_A);
		acc[2] = (i2c_smbus_read_byte_data(file,OUT_Z_H_A)<<8) | i2c_smbus_read_byte_data(file,OUT_Z_L_A);

                for (int j =0 ; j<3; j++)
                {
                    if(acc[j] >= 32768)
                    {
                         acc[j] =  65538.0f - acc[j];
                         acc[j] = -acc[j];
                    }
                }
                
                acc[0] = (acc[0]*90.0f/4096.0f);
                acc[1] = (acc[1]*90.0f/4096.0f);
                acc[2] = (acc[2]*90.0f/4096.0f);

                if(acc[2]>=0){acc[0] = 180.0f - acc[0];}

		std::cout << std::fixed  << "Acc "  << " X: " << acc[0] << " Y: " << acc[1] << " Z: " << acc[2] << std::endl;

		if (acc[0]<=90 || acc[0]<=(-90)) target = ( 0.5555556f * acc[0]+50.0f);
		else target = 100.0f;
        }
		pwm2.setDutyCycle(target);  // Set the duty cycle as a percentage
		std::cout << "eQEP " << " avt = " << avt << std::endl;
		std::cout << "eQEP " << " dx/dt = " << vel << std::endl;
		std::cout << "eQEP " << " target = " <<100.0f - target << std::endl;
		std::cout << "eQEP " << " feedback = " << (100.0f - (float)(vel*100/30000)) << std::endl;
   }
	    // Return success
    return 0;
}
