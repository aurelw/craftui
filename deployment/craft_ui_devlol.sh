#!/bin/bash

### BEGIN INIT INFO
# Provides:          craftui
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start script for devlock locksyste.
# Description:       start script for devlock locksyste.
### END INIT INFO

USER="lol"
CRAFTUI_ROOT="/home/$USER/packages/craftui/"
LOLSHIELD_ROOT="/home/$USER/packages/lolshield/"
CRAFTUI_CONFIG="$CRAFTUI_ROOT/build/bin/devlol_window.xml"

MQTT_BROKER="192.168.7.2"
LOGFILE="$CRAFTUI_ROOT/clients/events.log"


case "$1" in

    start)

        # start craftui and clients
        su $USER <<EOF
        echo "Starting craftui and clients"
        cd $CRAFTUI_ROOT
        screen -S craftui -d -m bash -c "./build/bin/craftui -c $CRAFTUI_CONFIG --noui"
        screen -S craftui_client -d -m bash -c "./clients/craftui_devlol.py"
        screen -S craftui_mqtt -d -m bash -c "./clients/craftui_mqtt_pub.py $MQTT_BROKER"
        screen -S craftui_log -d -m bash -c "./clients/craftui_print_event.py --logfile $LOGFILE --short"

        # start the lolshield
        cd $LOLSHIELD_ROOT
        echo "Starting lolshield... (10 sec)"
        # wait some time for the arduino to come up
        sleep 10
        screen -S lolshield -d -m bash -c "./lolcontrol/lolshield_mqtt.py --host $MQTT_BROKER"
EOF

    ;;

    stop)
        if [ "`pgrep craftui`" ]
        then
            echo "Stop craftui and clients"
            kill -9 `pgrep craftui`
        fi
        if [ "`pgrep lolshield`" ] 
        then
            echo "Stop lolshield"
            kill -9 `pgrep lolshield`
        fi

    ;;

esac

exit 0

