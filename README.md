A single cell Li-Ion/Li-Po/LiFePO4 MPPT solar charger
===========================================================



Roadmap for the 1.0.0 firmware
-------------------------------------

- fw: enable setting of the solar panel MPP voltage
- fw: enable setting of the battery float voltage (3.6V/4.2V)
- fw: enable setting of the battery UVLO
- fw: save the settings in a nonvolatile storage
- fw: locking/unlocking settings which may cause damage (battery voltages, etc.)


Firmware changelog (1.0.0-dev)
-------------------------------------

- fw: added I2C timeouts to fix communication errors with the STC3100 battery gauge/monitor
- fw: interface was changed to protocol buffers (using the nanopb library) and defined as a proto file
- fw: firmware has been split into multiple files
- fw: added IWDG watchdog
- fw: added undervoltage shutdown for the Vbus converter (hardcoded now)
- fw: added NTC battery temperature reading


TODO
------------

- fw: add periodic scan of the Vin_reg to fnd an optimal MPP voltage
- fw: add perturb and observe algo for setting of the MPP voltage
- fw: add status led
- fw: add stop mode when not charging (PWM is off)
