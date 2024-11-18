#!/bin/bash

TOPIC_CONDITION_UPDATE="/condition_update"
TOPIC_BEHAVIOR_TREE="/behavior_tree"

if [ $# -eq 2 ]; then
    FILE_XML="$1"
    CONDITION="$2"
else
    echo "Usage: $0 [mission_xml_path] [dust|rocks]"
    exit 1
fi

# Setting initial conditions
if [ "$CONDITION" == "dust" ]; then
    CONDITION_JSON="{\"condition\": \"OnDust\", \"value\": true}"
elif [ "$CONDITION" == "rocks" ]; then
    CONDITION_JSON="{\"condition\": \"OnDust\", \"value\": true}"
else
    echo "Specified condition not valid. Use 'dust' or 'rocks'."
    exit 1
fi

ros2 topic pub --once $TOPIC_CONDITION_UPDATE std_msgs/msg/String "{data: '$CONDITION_JSON'}"

# Setting mission
if [ ! -f "$FILE_XML" ]; then
    echo "Il file $FILE_XML non esiste."
    exit 1
else
    XML_CONTENT=$(cat "$FILE_XML")

    ros2 topic pub --once $TOPIC_BEHAVIOR_TREE std_msgs/msg/String "{data: '$XML_CONTENT'}"
fi
