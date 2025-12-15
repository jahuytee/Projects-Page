#!/usr/bin/env bash
#
#   Enable I2C and GPIO 1 on the Pi (for Ubuntu 22.04, Spring 2025)
#
#   05/29/25 Created
#   05/30/25 Kicking the daemon
#   05/30/25 Also enabling the I2C bus.
#


#
# Update the boot configuration to prevent the Pi from probing GPIO 0
# and 1.  These control HATs and at boot time, this I2C interface is
# interrogated to look for an EEPROM that identifies the attached
# board and allows automatic setup of the GPIOs (and optionally, Linux
# drivers).
#
# We don't use HATs and instead want to use these pins normally.  Edit
# /boot/config.txt to include 'force_eeprom_read=0'
#
file=/boot/firmware/config.txt
sudo cp -p $file ${file}-$(date +%Y%m%d%H%M%S)
if grep -q force_eeprom_read $file ; then
    # Edit the setting.
    echo "Editing $file"
    sudo sed $file -i -e "s/force_eeprom_read=.*/force_eeprom_read=0/"
else
    # Append.
    echo "Appending to $file"
    sudo tee -a $file <<EOF > /dev/null

# Prevent the firmware from trying to read an I2C HAT EEPROM at powerup.
# This frees up GPIO 0,1 pins (ID_SD & ID_SC) for other uses.
force_eeprom_read=0
EOF
fi


#
# Update the pigpio daemon to allow control of GPIO 0 and 1.
#
file=/lib/systemd/system/pigpiod.service
sudo cp -p $file ${file}-$(date +%Y%m%d%H%M%S)
# Edit the setting.
echo "Editing $file"
sudo sed $file -i -e "s/pigpiod -l.*/pigpiod -l -x -1/"


#
# Reload and kick the pigpiod for this change to take effect.
#
sudo systemctl daemon-reload
sudo systemctl restart pigpiod


#
# Also enable the I2C bus!
#
sudo raspi-config nonint do_i2c 0
