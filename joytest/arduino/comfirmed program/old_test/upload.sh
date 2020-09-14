cd accelerate_control4
arduino --port /dev/gem2/accbrake --upload accelerate_control4.ino 

cd ../speed_measure2
arduino --port /dev/gem2/speedsens --upload speed_measure2.ino

cd ../Steering_motor_with_Estop6
arduino --port /dev/gem2/steering --upload Steering_motor_with_Estop6.ino 

cd ../rev_and_lamp2
arduino --port /dev/gem2/gearandlamp --upload rev_and_lamp2.ino

cd ..
