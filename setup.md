 How to use Airsim in Ubuntu

# How to build Airsim in Ubuntu

- Initialize
    1. Install Ubuntu 18.04
    2. Install GPU driver
        
        ```bash
        # In my case : 
        ubuntu-drivers devices
        # == /sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 ==
        # modalias : pci:v000010DEd00001E81sv00001462sd0000372Dbc03sc00i00
        # vendor   : NVIDIA Corporation
        # driver   : nvidia-driver-525 - distro non-free
        # driver   : nvidia-driver-510 - distro non-free
        # driver   : nvidia-driver-515 - distro non-free
        # driver   : nvidia-driver-450-server - distro non-free
        # driver   : nvidia-driver-525-server - distro non-free
        # driver   : nvidia-driver-470-server - distro non-free
        # driver   : nvidia-driver-530 - distro non-free
        # driver   : nvidia-driver-515-server - distro non-free
        # driver   : nvidia-driver-470 - distro non-free recommended
        # driver   : xserver-xorg-video-nouveau - distro free builtin
        sudo apt-get install nvidia-driver-470
        ```
        
    3. Sign up in Epic Games([epicgames.com](http://epicgames.com/)) and connect account to your Github(lets you build Unreal Engine)
    4. Clone repositories - Unreal Engine must be 4.27
        
        ```bash
        cd
        sudo apt install git
        git clone https://github.com/<your account>/AirSim.git
        git clone -b 4.27 https://github.com/<your account>/UnrealEngine.git
        cd ~/AirSim/Unreal/Environments/Blocks && ./clean.sh
        ```
        
    5. Build Unreal Engine and Airsim
        
        ```bash
        cd ~/UnrealEngine
        ./Setup.sh
        ./GenerateProjectFiles.sh
        make -j10
        
        cd ~/AirSim
        ./setup.sh
        ./build.sh
        ```
        
    6. Install mono(lets you run .exe files on Linux) and make symlink
        
        ```bash
        sudo apt install ca-certificates gnupg
        sudo gpg --homedir /tmp --no-default-keyring --keyring /usr/share/keyrings/mono-official-archive-keyring.gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
        echo "deb [signed-by=/usr/share/keyrings/mono-official-archive-keyring.gpg] https://download.mono-project.com/repo/ubuntu stable-bionic main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
        sudo apt update
        sudo apt install mono-devel
        sudo ln -s /usr/bin/mono /bin/mono && sudo ln -s /usr/lib/mono /lib/mono
        ```
        
    7. Build Airsim “blocks” project
        
        ```bash
        sudo ~/UnrealEngine/Engine/Binaries/ThirdParty/Mono/Linux/bin/mono ~/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool.exe Development Linux -Project=/home/<your path>/AirSim/Unreal/Environments/Blocks/Blocks.uproject -TargetType=Editor -Progress
        ```
        
    8. Run UE4Editor from command line and select ~/Airsim/Unreal/Environments/Blocks/Blocks.uproject
        
        ```bash
        ~/UnrealEngine/Engine/Binaries/Linux/UE4Editor
        ```
        
    9. Error shows up "This project was made with a different version of the Unreal Engine." → Click "More Options" > "Skip conversion"
    If error show again, delete Intermediate folder in ~/Airsim/Environment/Blocks/ and try again
    10. Press Play button once to create settings.json file
- Build
    
    ```bash
    #!/usr/bin/env bash
    
    cd ~/AirSim
    ./setup.sh
    ./build.sh
    cd ~/AirSim/Unreal/Environments/Blocks
    ./clean.sh
    ~/UnrealEngine/Engine/Binaries/ThirdParty/Mono/Linux/bin/mono ~/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool.exe Development Linux -Project=/home/jihwan/AirSim/Unreal/Environments/Blocks/Blocks.uproject -TargetType=Editor -Progress -maxthreads=8
    ~/UnrealEngine/Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject
    ```
    

# How to initialize & change project settings

~/Documents/Airsim/settings.json

```json
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone0"  : { "VehicleType": "SimpleFlight", "AllowAPIAlways": true, "RC": {"RemoteControlID":   0, "AllowAPIWhenDisconnected": true}, "X":  5, "Y":  5, "Z": -2 },
    "Drone1"  : { "VehicleType": "SimpleFlight", "AllowAPIAlways": true, "RC": {"RemoteControlID":   0, "AllowAPIWhenDisconnected": true}, "X": 10, "Y":  5, "Z": -2 },
    "Drone2"  : { "VehicleType": "SimpleFlight", "AllowAPIAlways": true, "RC": {"RemoteControlID":   0, "AllowAPIWhenDisconnected": true}, "X": 15, "Y":  5, "Z": -2 },
    "Drone3"  : { "VehicleType": "SimpleFlight", "AllowAPIAlways": true, "RC": {"RemoteControlID":   0, "AllowAPIWhenDisconnected": true}, "X": 20, "Y":  5, "Z": -2 },
    "Drone4"  : { "VehicleType": "SimpleFlight", "AllowAPIAlways": true, "RC": {"RemoteControlID":   0, "AllowAPIWhenDisconnected": true}, "X": 25, "Y":  5, "Z": -2 }
  }
}
```

# How to control vehicles by API

- Install python3 and libraries
    
    ```bash
    sudo apt install python3-pip
    python3 -m pip install --upgrade pip
    pip3 install numpy
    pip3 install msgpack-rpc-python
    pip3 install scikit-build
    pip3 install cmake
    pip3 install airsim
    ```
    
- Control vehicles by python api
    
    [Core APIs - AirSim](https://microsoft.github.io/AirSim/apis/)
    
    ```python
    import airsim
    import cv2
    import numpy as np
    import os
    import pprint 
    import tempfile
    
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Start control
    drone_names = ["Drone" + str(i) for i in range(5)]
    for drone in drone_names:
        client.enableApiControl(True, drone)
    for drone in drone_names:
        client.armDisarm(True, drone)
    
    # Takeoff command must be given before the flight command
    airsim.wait_key('Press any key to takeoff')
    takeoff_tasks = [client.takeoffAsync(vehicle_name=drone) for drone in drone_names]
    for task in takeoff_tasks:
        task.join()
    
    # Flight control command
    airsim.wait_key('Press any key to move vehicles')
    move_tasks = [client.moveToPositionAsync(0, 0, -int(drone[-1])-5, 5, vehicle_name=drone) for drone in drone_names]
    for task in move_tasks:
          task.join()
    
    # Finish control
    for drone in drone_names:
        client.armDisarm(False, drone)
    client.reset()
    for drone in drone_names:
        client.enableApiControl(False, drone)
    ```
    

# Issues

### Unreal editor is slow when it is not the active window

- Go to Edit/Editor Preferences, select "All Settings" and type "CPU" in the search box. It should find the setting titled "Use Less CPU when in Background", and you want to uncheck this checkbox.

# How to change view manually

1. Press m
2. WASD, arrow keys(←→↑↓), PageUp/PageDown will change your view

# How to use ROS

[https://github.com/microsoft/AirSim/blob/main/docs/airsim_ros_pkgs.md](https://github.com/microsoft/AirSim/blob/main/docs/airsim_ros_pkgs.md)

Frame,CPU,Render,GPU,RHIT
