@echo off
set TARGET=%homepath%\Documents\AirSim\
cd %cd%

copy settings.json %TARGET%
copy settings_car.json %TARGET%
copy settings_drone.json %TARGET%
copy switch_car.bat %TARGET%
copy switch_drone.bat %TARGET%
@pause
