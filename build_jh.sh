#!/bin/sh

cd ~/AirSim
./setup.sh
./build.sh
cd ~/AirSim/Unreal/Environments/Blocks
./clean.sh
~/UnrealEngine/Engine/Binaries/ThirdParty/Mono/Linux/bin/mono ~/UnrealEngine/Engine/Binaries/DotNET/UnrealBuildTool.exe Development Linux -Project=/home/jihwan/AirSim/Unreal/Environments/Blocks/Blocks.uproject -TargetType=Editor -Progress
~/UnrealEngine/Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject
