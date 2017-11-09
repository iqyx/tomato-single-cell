A single cell Li-Ion/Li-Po/LiFePO4 MPPT solar charger
===========================================================


Changelog (0.1.0-dev)
-------------------------------------

- fw: added I2C timeouts to fix communication errors with the STC3100 battery gauge/monitor
- fw: interface was changed to protocol buffers (using the nanopb library) and defined as a proto file
- fw: firmware has been split into multiple files


Roadmap for the 1.0.0 firmware
-------------------------------------

- fw: enable setting of the solar panel MPP voltage
- fw: settable undervoltage shutdown for the Vbus converter
- fw: enable setting of the battery float voltage (3.6V/4.2V)


TODO
------------

- fw: add periodic scan of the Vin_reg to fnd an optimal MPP voltage
- fw: add perturb and observe algo for setting of the MPP voltage
- fw: add battery temperature monitoring (NTC)
- fw: add status led
- fw: add stop mode when not charging (PWM is off)
