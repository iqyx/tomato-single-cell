A single cell Li-Ion/Li-Po/LiFePO4 MPPT solar charger
===========================================================

TODO
------------

- fw: split the PoC firmware into multiple parts
- fw: check for I2C errors durring reading/writing to the STC3100 battery gauge
- fw: software reset the I2C peripheral if an unrecoverable error occurs
- fw: define the interface in a IDL language (protobuf)
- fw: change the interface to protobufs
- fw: enable setting of the battery float voltage (3.6V/4.2V)
- fw: enable setting of the solar panel MPP voltage
- fw: add periodic scan of the Vin_reg to fnd an optimal MPP voltage
- fw: add perturb and observe algo for setting of the MPP voltage
- fw: add battery temperature monitoring (NTC)
- fw: add status led
- fw: add stop mode when not charging (PWM is off)
