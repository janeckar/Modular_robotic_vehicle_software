#!/bin/bash

# CPU temperature
num_cpu_temp=`cat /sys/class/thermal/thermal_zone0/temp`
cpu_length=${#num_cpu_temp}
integer_cpu_temp=${num_cpu_temp:0:$((cpu_length - 3))}
floating_cpu_temp=${num_cpu_temp: -3}
echo "CPU_TEMP: ${integer_cpu_temp},${floating_cpu_temp}°C"

# GPU temperature
gpu_text_temp=`vcgencmd measure_temp` # output: "temp=44.5'C"
gpu_format_temp=${gpu_text_temp:5}
gpu_integer=${gpu_format_temp:0:2}
gpu_floating=${gpu_format_temp: -3:1}
echo "GPU_TEMP: ${gpu_integer},${gpu_floating}°C"