#!/bin/bash
# My first script

echo "Configuring pins"
echo "Export slots"
export SLOTS=/sys/devices/bone_capemgr.9/slots
echo "Slots directory :  " $SLOTS
echo "Exporting epwmss"
echo PyBBIO-epwmss0 > $SLOTS
echo "Exporting eqep0"
echo PyBBIO-eqep0 > $SLOTS
echo bone_pwm_P9_22 > $SLOTS
echo bone_pwm_P9_21 > $SLOTS
echo am33xx_pwm > $SLOTS

