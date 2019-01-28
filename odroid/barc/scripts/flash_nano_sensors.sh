rm -rf ~/barc/arduino/.arduino_nano328_node_sensors 
if [ ! -d ~/barc/arduino/.arduino_nano328_node_sensors ]; then
    mkdir -p ~/barc/arduino/.arduino_nano328_node_sensors/src
fi

if [ ! -L ~/barc/arduino/.arduino_nano328_node_sensors/lib ]; then
    ln -s ~/sketchbook/libraries_testing ~/barc/arduino/.arduino_nano328_node_sensors/lib
fi

cd ~/barc/arduino/.arduino_nano328_node_sensors
cp ../arduino_nano328_node_sensors/arduino_nano328_node_sensors_11.ino src/; 
#cp ../arduino_nano328_node_sensors/ldr_echo_rfid.ino src/; 
#cp ../arduino_nano328_node_sensors/arduino_nano328_node_sensors.ino src/; 
ano clean; ano build -m nano328 -f='-std=c++11 -Os'; 
ano upload -m nano328 -p /dev/ttyUSB1; 
cd -
