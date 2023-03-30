#!/bin/bash
sudo -S cp autoinstall/cigritous_autostart.sh /usr/local/bin/

sudo -S chmod +x /usr/local/bin/cigritous_autostart.sh

sudo -S cp autoinstall/cigritous.service /etc/systemd/system/

sudo systemctl start cigritous.service
sudo systemctl status cigritous.service
sudo systemctl enable cigritous.service