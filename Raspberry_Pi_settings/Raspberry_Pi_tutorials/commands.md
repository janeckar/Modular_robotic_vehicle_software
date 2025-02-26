## core temperature:
to get result in degrees divide by 1000
    
    cat /sys/class/thermal/thermal_zone0/temp

## Permissions

    In order to use the GPIO ports, your user must be a member of the gpio group. The default user account is a member by default, but you must add other users manually using the following command:
```
    sudo usermod -a -G gpio <username>
```