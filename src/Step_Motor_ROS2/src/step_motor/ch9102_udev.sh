echo  'KERNEL=="ttyACM*",      ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="5B1F028280", MODE:="0777", GROUP:="dialout", SYMLINK+="motor_serial"' >/etc/udev/rules.d/step_motor1.rules
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="5B1F028280", MODE:="0777", GROUP:="dialout", SYMLINK+="motor_serial"' >/etc/udev/rules.d/step_motor2.rules


service udev reload
sleep 2
service udev restart


