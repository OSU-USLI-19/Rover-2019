 #! /bin/sh cd /sys/devices/platform/bone_capemgr File=slots if grep -q "Override Board Name,00A0,Override Manuf,univ-emmc" "$File";  then    
    cd
    echo "\n Pin configuration available"
    #echo "\n UART 4 configuration p9.11 and p9.13"
    #sudo config-pin P9.11 uart
    #sudo config-pin -q P9.11
    #sudo config-pin P9.13 uart
    #sudo config-pin -q P9.13
    echo "\n UART 1 configuration p9.26 and p9.24"
    sudo config-pin P9.24 uart
    sudo config-pin -q P9.24
    sudo config-pin P9.26 uart
    sudo config-pin -q P9.26
    #echo "\n UART 5 configuration p8.38 and p8.37"
    #sudo config-pin P8.38 uart
    #sudo config-pin -q P8.38
    #sudo config-pin P8.37 uart
    #sudo config-pin -q P8.37
    echo "\n UART configuration end"  else    
    echo "Oops!!configuration is not available"
    echo "Please check uEnv.txt file and only disable HDMI" fi