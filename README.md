# Robocon2026

## hardware setup
### parts
1. Motor -> [D5065 270KV](https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html#d5065-270kv)
    - ![alt text](documentationMaterial\D5065_specs.png)
    - estimate loadless rpm @ 14.8v = 3996rpm / 66 turns/s (:skull:)  
    - target rpm = 800rpm / 13turns/s
2. Driver -> Odrive S1
3. Battery -> [Elements 6200mAh 100C 14.8V Lipo](https://hobbysquare.com.sg/collections/battery-4s-14-8v/products/elements-6200mah-100c-14-8v-lipo-battery-for-rc-car-hardcase) 
    - estimate continuous current draw = ~300A
### wiring
!todo

setting up motor wires  
1. trim to a clean slate
2. if you trim quite short, and see strands of copper wire instead of a hemogenous block with solder, continue to step 3, else skip to step 4.
3. remove the enamel from the copper strands. you can either do this by sanding the wire until the enamel is gone, or the [solder pot method](https://www.youtube.com/watch?v=pByM6vsicaE). if you dont do this, you will face unbalanced phase resistances.
4. [crimp a ferrule](https://www.youtube.com/watch?v=cqyYBua4pX4&t=99s) (or dont, but i like ferrures because youll never get stray wires sticking out the side, and i personally and unscientifically think the mechanical connection is stronger and its less likely to fall out)
5. stick the boi into the a,b,c screw terminals
6. the exact wire that you stick into each terminal doesnt matter. however, if the motor is spinning the wrong way (ie. forward is backwards) swap any 2 wires


### motor config
1. use the odrive [gui](https://gui.odriverobotics.com/configuration) (which is just a nicer version of odrivetool, which can also be found in the venv of src)
2. in the power source tab
    - DC bus overvoltage level = 18v (what is the maximum voltage our battery should be at)
    - DC bus undervoltage level = 12v (what is the minimum voltage our battery should be at)
    - DC max positive current = 100A (theroedically, could be like 6.2Ah * 100C = 620A, x50% for margin, 300A. but yknow be safe)
    - DC max negative current = -5A (again, therodeically, charge rate of the battery is 1-3C, max 5C, so could be like 6.2Ah*5C = 18.6A, x50% for margin, -9A. but again yknowbesafe)  
3. in the motor tab
    - choose d5065
    - motor calib current = 20A
    - motor calib voltage = 1.5v
    - lock-in spin current = 33A
    - (for more infomation on calibration params read [this](https://docs.odriverobotics.com/v/latest/articles/motor-parameters.html#motor-parameters-troubleshooting))
    - use thermistor enabled
4. in encoder tab 
    - choose amt21xB-V-OD
5. in control mode tab
    - soft velocity limit = 40 turns / s
    - hard velocity limit = 50 turns / s (about 3000rpm so)
    - tbh once we move to the drivetrain **these values should change**
6. in interfaces
    - turn on watchdog timer (default is 1s but when moving to drivetrain maybe drop it to 0.5s?)
    - turn on CAN
        - Ensure that baud rate is set to 250000
    - for now, keep to default, but when implementing can frfr, here are some pointers,
    - node id identifies the different motors in the bus. you can assign the id dynamically by matching id to serial number (each motor has a unchanging serial number) thorugh can interface, or just statically assign through the gui. bothwork, read [this](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#discovery-addressing) for more info
    - definately turn on heartbeat, incase the wire dies for whatever reason we can atleast detect it
    - consider turning on error cyclic messages
7. in apply and calibrate, 
    - click all the buttons sequentially.
    - if you only edited limits or stuff like that, if it says "takes effect: immediately" in the docs for that property, you only need to press "apply new configuration" button
    - if you get phase unbalanced, its time to fix the wires baby (read hardware setup section)
8. all dones
9. if for some reason we ever do it with a python script instead, here the params: 
    ```python
    odrv = odrv0
    odrv.config.dc_bus_overvoltage_trip_level = 18
    odrv.config.dc_bus_undervoltage_trip_level = 12
    odrv.config.dc_max_positive_current = 90
    odrv.config.dc_max_negative_current = -1
    odrv.config.brake_resistor0.enable = False
    odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
    odrv.axis0.config.motor.pole_pairs = 7
    odrv.axis0.config.motor.torque_constant = 0.030629629629629628
    odrv.axis0.config.motor.current_soft_max = 65
    odrv.axis0.config.motor.current_hard_max = 85
    odrv.axis0.config.motor.calibration_current = 20
    odrv.axis0.config.motor.resistance_calib_max_voltage = 1.5
    odrv.axis0.config.calibration_lockin.current = 33
    odrv.axis0.motor.motor_thermistor.config.enabled = True
    odrv.axis0.motor.motor_thermistor.config.r_ref = 10000
    odrv.axis0.motor.motor_thermistor.config.beta = 3435
    odrv.axis0.motor.motor_thermistor.config.temp_limit_lower = 110
    odrv.axis0.motor.motor_thermistor.config.temp_limit_upper = 130
    odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
    odrv.axis0.controller.config.vel_limit = 10
    odrv.axis0.controller.config.vel_limit_tolerance = 1.2
    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf
    odrv.axis0.trap_traj.config.accel_limit = 10
    odrv.axis0.controller.config.vel_ramp_rate = 10
    odrv.can.config.protocol = Protocol.SIMPLE
    odrv.can.config.baud_rate = 0
    odrv.axis0.config.can.node_id = 63
    odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
    odrv.axis0.config.can.encoder_msg_rate_ms = 10
    odrv.axis0.config.can.iq_msg_rate_ms = 10
    odrv.axis0.config.can.torques_msg_rate_ms = 10
    odrv.axis0.config.can.error_msg_rate_ms = 100
    odrv.axis0.config.can.temperature_msg_rate_ms = 1000
    odrv.axis0.config.can.bus_voltage_msg_rate_ms = 1000
    odrv.axis0.config.enable_watchdog = True
    odrv.axis0.config.watchdog_timeout = 1
    odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
    odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
    odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.CUI_AMT21_EVENT_DRIVEN
    odrv.config.enable_uart_a = True
    odrv.config.gpio7_mode = GpioMode.UART_A
    odrv.config.gpio6_mode = GpioMode.UART_A
    odrv.config.uart_a_baudrate = 115200
    ```


## software setup
pc side python script is unfortunately **windows only** (blame kevin why he use xinput)  
you have to build the virtual enviroment before launching scripts to make sure you have all the dependences (odrive, pyserial, [XInput-Python](https://github.com/Zuzu-Typ/XInput-Python/tree/master))
1. launch terminal in this ```Robocon2026``` folder
2. type the following
    ```powershell
    py -m venv .venv
    .venv\Scripts\activate
    py -m pip install -r requirements.txt
    ```