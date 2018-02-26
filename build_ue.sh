./build.sh
cd ~/GitHub/UnrealEngine
./GenerateProjectFiles.sh -game ~/GitHub/AirSim/Unreal/Environments/Blocks/Blocks.uproject
cd ~/GitHub/AirSim/Unreal/Environments/Blocks
make Blocks
