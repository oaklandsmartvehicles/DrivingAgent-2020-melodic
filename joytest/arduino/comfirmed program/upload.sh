cd accelerate_control7
arduino --board arduino:avr:mega:cpu=atmega2560 --port /dev/gem2/accbrake --upload accelerate_control7.ino 

cd ../speed_measure8
arduino --board arduino:avr:uno --port /dev/gem2/speedsens --upload speed_measure8.ino

cd ../Steering_motor_with_Estop7
arduino --board arduino:avr:uno --port /dev/gem2/steering --upload Steering_motor_with_Estop7.ino 

cd ../rev_and_lamp3
arduino --board arduino:avr:uno --port /dev/gem2/gearandlamp --upload rev_and_lamp3.ino

cd ..
