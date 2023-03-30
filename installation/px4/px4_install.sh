#!/bin/bash

echo "Installing PX4-ROS2 environment"

sudo -S apt-get install -y openjdk-11-jre

cd ~/

git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen

cp cigritous_ws/src/installation/px4/gradle-wrapper.properties ~/Fast-RTPS-Gen/gradle/wrapper

cd ~/Fast-RTPS-Gen 

./gradlew assemble && sudo env "PATH=$PATH" ./gradlew install