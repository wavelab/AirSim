@echo off
cd %cd%
del settings.json
copy settings_car.json settings.json
@pause