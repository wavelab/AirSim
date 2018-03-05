@echo off
cd %cd%
del settings.json
copy settings_drone.json settings.json
@pause