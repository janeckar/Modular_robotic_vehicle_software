# Setup

## Setup networking
To setup internet connection on raspberrypi in a way that the configuration can be used in any enviroment. One of the good ways to make the setup is by creating hotspot on main PC from which the robot will be driven.
  
For settup of the networking, the netplan will be used as main tool. To make the configuration it is needed to create configuration file in **/etc/netplan/** directory. Ubuntu OS is using this by default on Ubuntu server to init cloud but it is not needed so remove the **50-init-cloud.yaml**. And create your own netplan configuration file **01-netcfg.yaml**.

Fill the **01-netcfg.yaml** file with the following contents and change the SSID of the network and WIFI password. 

The interface of the WIFI can vary so be careful and check the name of the interface with command _"ip address"_. Be careful when editing yaml file format is very sensitive for size of the tabs which should be 2 spaces long. 

### 01-netcfg.yaml file
```
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: yes
      access-points:
        "SSID_REPLACE":
          password: "password_REPLACE"
```

### Making the hostname of the Raspberry Pi visible on network 
After the Raspberry Pi is successfully connected to the hotspot, it is good to setup the hostname to be visible on network to other devices on network.

So the other devices can see the hostname of the Raspberry Pi it is needed to start avahi-deamon which through multicast DNS sends over local network the hostname of our Raspberry Pi. 

- installing avahi-deamon and starting it
```
$ sudo apt install avahi-daemon
$ sudo systemctl start avahi-daemon
# check the status of avahi-daemon
$ sudo systemctl status avahi-daemon
```

Set your hostname to your choice and at the end add suffix ".local".

After this operation it should be possible to connect to the Raspberry Pi via ssh

```
# cli
$ ssh username@hostname.local
```

- to get hostname of your Raspberry Pi type
```
$ hostname
```

## Useful library for scanning gpio pins
- install gpiod
- console apps:
    - gpiolist ... lists gpio chips
    - gpioinfo ... lists all gpio lines with usage status
    - other cli tools for manipulating the gpio

## Setup Enviroment

1. changing options for raspberrypi dtoverlay
```
file: /boot/firmware/config.txt or /boot/config.txt
# at the end add
dtoverlay=gpio-shutdown,gpio_pin=21
```
- turning off SPI bus whether it is not needed
```
file: /boot/firmware/config.txt or /boot/config.txt
# at the beggining of the file change the spi option to off
dtparam=spi=off
```

## Install nessesary libraries

### 1. Install WiringPi
```
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi

# build the package
./build debian
mv debian-template/wiringpi-3.0-1.deb .

# install it
sudo apt install ./wiringpi-3.0-1.deb
```

### 2. setup the gpio device to be accesible for user without root access
- add these lines to your .bashrc or .bash_aliases because these 
changes won't persist after reboot

- to setup the gpio device to be accessible by user we need to add the user to gpio group

- then it is needed to add the device to the gpio group and to do it persistently with use of udev device manager configuration rules files

- to check that the linux user is in gpio group, type

```
$ groups $USER | grep gpio
user : user gpio
# if the user is not in the gpio group, then add user to gpio group
$ usermod -aG gpio $USER
```

- create rule for udev in configuration directory **/etc/udev/rules.d/** name it as for example **10-gpiomem.rules**, fill the content of the file with the next line

```
SUBSYSTEM=="gpio", KERNEL=="gpiomem", GROUP="gpio", MODE="0660"
```

- apply the new rules for udev device manager with use of tool udevadm
```
$ sudo udevadm control --reload-rules && sudo udevadm trigger
```

- to check if everything is set, check gpio device access rights
```
$ ll /dev/gpiomem
# output should look like this
crw-rw---- 1 root gpio 999, 0 Jan 1 00:00 /dev/gpiomem
```

- if something is not right try to check the device info and try to look at this documentation: https://docs.oracle.com/en/operating-systems/oracle-linux/8/udev/udev-QueryingUdevandSysfs_prefixes_for_udevadm_information.html 

```
udevadm info --name=/dev/gpiomem
```
#### Add the gpio device to gpio group for one session only!
For this setting it will not persist.

```
$ sudo chgrp gpio /dev/gpiomem
$ sudo chmod g+rw /dev/gpiomem
```



## Cloning repository with ros2 packages
zero start the setup script
first rosdep init, rosdep update, rosdep install --from-paths src -i src 


