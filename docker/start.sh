#!/bin/bash

if [ "$vncpassword" = "" ]
then
    vncpassword="insecure"
fi

echo "export DISPLAY=:0" >> ~/.bashrc

while true; 
    do echo "still live"; 
    sleep 600; 
done