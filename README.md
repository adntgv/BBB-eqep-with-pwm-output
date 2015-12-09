
<h1>Gearmotor control by using accelerometer as an analog input</h1>

<h2>Description</h2>
The aim of our project was to control Pololu motor via accelerometer sensor. 

For that purpose were used:
<ul>
  <li>BeagleBone Black
  <li>Pololu accelerometer
  <li>Pololu Gearmotor 50:1 37D54L
  <li>Pololu JRK 21v3 controller
</ul>
<b>Using an eqep to read data from encoder</b>

We can check if we have required drivers on the BBB with a following command:

<p>$ ls /lib/firmware/ | grep -i qep
<p>Output should be as follows:

    PyBBIO-eqep0-00A0.dtbo
    
    PyBBIO-eqep1-00A0.dtbo
    
    PyBBIO-eqep2-00A0.dtbo
    
    PyBBIO-eqep2b-00A0.dtbo
    
If there are no such drivers, you should update the firmware of your BB Black
    
Now we will enable the driver

Note: epwmss has to be enabled before enabling eqep and other pwm modules

    $ echo PyBBIO-epwmss > /sys/devices/bone_capemgr.*/slots
    $ echo PyBBIO-eqep2b > /sys/devices/bone_capemgr.*/slots

You can use other eqep* drivers referencing to those files where used ports are listed:

https://github.com/Teknoman117/beaglebot/tree/master/encoders/dts

Test

    $ cd /sys/devices/ocp.*/*.epwmss/*.eqep/

    $ cat position You will see and output number of disposition.

Rotate and cat again

Automatization

For our project we used api provided in examples section of :

https://github.com/Teknoman117/beaglebot/tree/master/encoders/api/c++

Shell script  exports.sh eports needed pin configurations for our project. 
File test.cpp is the main file where control is implemented. You can customize those files depending on your needs. 

<b>Reading accelerometer position</b>

Data from accelerometer is acquired through I2C ports. The model of used accelerometer is LSM303D from Pololu. The acquired data from accelerometer is transformed into degrees for easier interaction. The source code for that is also provided in test.cpp 
As the data is stored in two chunks of 8 bits for Low and High bytes for each axis we need to read them separately and merge into one number like:

    acc[0]=(i2c_smbus_read_byte_data(file,OUT_X_H_A)<<8)|i2c_smbus_read_byte_data(file,OUT_X_L_A);

Then we need to separate negative and positive values. Negative values are subtracted from the maximum, so we can manage signs as follows:

    if(acc[j] >= 32768) { acc[j] = 65538.0f - acc[j]; acc[j] = -acc[j];


After that we convert into degrees:

    acc[0] = (acc[0]*90.0f/4096.0f);

Finally we have to teach our program bottom from the top:

    if(acc[2]>=0){acc[0] = 180.0f - acc[0];}


<b>PWM control</b>

For pwm output on pins BBB has a specialized module that is activated the same way as eqep. 
We just need to transform values of an accelerometer to needed voltage:

    if (acc[0]<=90 || acc[0]<=(-90)) target = ( 0.5555556f * acc[0]+50.0f);
    pwm2.setDutyCycle(target);

<b>Amplification</b>

The operational range of feedback receiver of the controller was between 0-5 volts. 
As the BB pwm can supply maximum of 3.3 V we need to amplify it. The easiest way was to use operational amplifiers.
After straightforward calculations we used non-inverting amplification circuit with the 3.3kOhms and 6.8 kOhms. 

<b>Control of Motor</b>

Control of a motor is realized through a specialized jrk 21v3 from Pololu, 
which includes GUI control tool for Windows where all configurations can be changed like PID and others. 
Controller can work in several modes like Analog or Serial. For our project we used analog mode. 
The target and feedback were acquired from BB. We have successfully controlled a motor in open loop mode. 
For closed loop correct PID configuration is required.
