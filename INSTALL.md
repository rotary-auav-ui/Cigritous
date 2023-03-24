# Cigritous Ground Module Install

## Prerequisites
1. [Arduino IDE](https://www.arduino.cc/en/software)
2. [ESP32-Arduino IDE](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)

## Installation
1. Clone the repository

`git clone --recurse-submodules -b ground-module https://github.com/rotary-auav-ui/cigritous.git`

2. Copy all the folders in `libraries` to 

`<your username>\Documents\Arduino\libraries` (for Windows user)
`~/arduino/sketchbook/libraries` (for Linux user)

3. Open Arduino IDE

4. Configure the nodes and modules at `settings.h` according to your setup

5. Upload program to device by clicking right direction at left-top

6. Repeat step 4 and 5 for every nodes
