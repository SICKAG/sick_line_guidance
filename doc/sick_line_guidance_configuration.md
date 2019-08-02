# sick_line_guidance configuration

The ROS drivers for MLS and OLS devices are configured by launch and yaml files. You can modify the default configuration by editing these files.
Please note, that an invalid or improper configuration may cause errors and unexpected results.

The basic configuration uses the default values for OLS and MLS devices. All changes should be tested carefully.

To start the MLS resp. OLS driver, just launch sick_line_guidance with a yaml-configuration file:
```bash
roslaunch sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_mls.yaml   # start MLS driver
roslaunch sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols10.yaml # start OLS10 driver
roslaunch sick_line_guidance sick_line_guidance.launch yaml:=sick_line_guidance_ols20.yaml # start OLS20 driver
```

The launch file "sick_line_guidance.launch" configures the common driver settings, f.e. the ros nodes to start and the ros topics to be used by 
sick_line_guidance. Unless special purposes, these settings do not require customization.

The 3rd argument to roslaunch provides the driver with a yaml-file: sick_line_guidance_mls.yaml for MLS resp. sick_line_guidance_ols10.yaml for OLS10
or sick_line_guidance_ols20.yaml for OLS20.
These two files contain the settings of the can devices and may require customization.

## yaml configuration for MLS

This is the default configuration for a MLS device in file mls/sick_line_guidance_mls.yaml:
```yaml
bus:
  device: can0
  driver_plugin: can::SocketCANInterface
  master_allocator: canopen::SimpleMaster::Allocator
sync:
  interval_ms: 10
  overflow: 0
nodes:
  node1:
    id: 0x0A # CAN-Node-ID of can device, default: Node-ID 10=0x0A for OLS and MLS
    eds_pkg: sick_line_guidance # package name for relative path
    eds_file: SICK-MLS.eds      # path to EDS/DCF file
    publish: ["1001","1018sub1","1018sub4","2021sub1!","2021sub2!","2021sub3!","2021sub4!","2022!"]
    # MLS: 1001 = error register, 1018sub1 = VendorID, 1018sub4 = SerialNumber, TPDO1 = 0x2021sub1 to 0x2021sub4 and 0x2022
    
    # sick_line_guidance configuration of this node:
    sick_device_family: "MLS"              # can devices of OLS10, OLS20 or MLS family currently supported
    sick_topic: "mls"                      # MLS_Measurement messages are published in topic "/mls"
    sick_frame_id: "mls_measurement_frame" # MLS_Measurement messages are published frame_id "mls_measurement_frame"

    # device configuration of writable parameter by dcf_overlay: "objectindex": "parametervalue"
    # example: "2027": "0x01" # Object 2027 (sensorFlipped, defaultvalue 0x00) will be configured with value 0x01
    # dcf_overlay:
    #   "2028sub1": "0x01" # UseMarkers (0 = disable marker detection, 1 = enable marker detection), UINT8, DataType=0x0005, defaultvalue=0
    #   "2028sub2": "0x02" # MarkerStyle (0 = disable marker detection, 1 = standard mode, 2 = extended mode), UINT8, DataType=0x0005, defaultvalue=0
    #   "2028sub3": "0x01" # FailSafeMode (0 = disabled, 1 = enabled), UINT8, DataType=0x0005, defaultvalue=0
    #   "2025":     "1000" # Min.Level, UINT16, DataType=0x0006, defaultvalue=600
    #   "2026":        "1" # Offset [mm] Nullpunkt, INT16, DataType=0x0003, defaultvalue=0
    #   "2027":     "0x01" # sensorFlipped, UINT8, DataType=0x0005, defaultvalue=0
    #   "2029":     "0x01" # LockTeach, UINT8, DataType=0x0005, defaultvalue=0
```

The following table describes these settings in detail:

| parameter name | default value | details |
| --- | --- | --- |
| bus / device | can0 | System name of can interface: net device driver for can hardware (f.e. pcan adapter) are installed to a net device named "can0". Check by system command "ifconfig -a"  |
| bus / driver_plugin | can:: SocketCANInterface | Implementation of interface SocketCAN, currently only can::SocketCANInterface is supported. |
| bus / master_allocator | canopen:: SimpleMaster:: Allocator | Implementation of CAN master (network manager), currently only canopen::SimpleMaster::Allocator is supported. |
| sync / interval_ms | 10 | Timeinterval for can sync messages in milliseconds (CAN master will send a nmt sync message each 10 ms) |
| sync / overflow | 0 | Length of can sync message (0: sync message does not contain any data byte) |
| nodes | node1 | Name of the first can node configured. If you're running more than 1 can device (multiple MLS or OLS devices), you can configure each device by providing a named section for each node. |
| **nodes / node1 / id** | **0x0A** | **CAN-Node-ID of can device (default: Node-ID 10=0x0A for OLS and MLS). Change this id, if your MLS uses a different CAN-ID. :exclamation:** |
| nodes / node1 / eds_pkg | sick_line_guidance | Name of sick_line_guidance ros package. Required to resolve filenames and should not be modified. |
| nodes / node1 / eds_file | SICK-MLS.eds | Electronic datasheet for your MLS device  |
| nodes / node1 / publish | ["1001", "1018sub1", "1018sub4", "2021sub1!", "2021sub2!", "2021sub3!", "2021sub4!", "2022!"] | List of published objects in the object dictionary of a can device. These objects are both published on ros topic <nodename>_<objectindex> and required by driver internal callbacks to handle PDO messages. Do not remove any PDO mapped objects from this list! |
| nodes / node1 / sick_device_family | "MLS" | Informs the ros driver that this can device is a MLS. Currently supported values are "OLS10", "OLS20" or "MLS". |
| nodes / node1 / sick_topic | "mls" | Name of the ros topic for publishing MLS measurement messages. If modified, the same topic must be set in file sick_line_guidance.launch for the sick_line_guidance_cloud_publisher node (param name="mls_topic_publish" type="str" value="mls"). Otherwise, measurement messges won't be transformed to PointCloud2 messsages. |
| nodes / node1 / sick_frame_id | "mls_measurement_frame" | Ros frame id of the MLS measurement messages. |
| **# nodes / node1 / dcf_overlay** |  | **Configuration of device parameter. To set an object in the object dictionary of a can device to a specific value at startup, append dcf_overlay entries with "objectindex": "parametervalue". This section is intensionally left empty to use default values. See operation manual for possible settings. :exclamation:** |

Device specific settings can be configured in section "nodes/node1/dcf_overlay". Example to activate marker detection in extended mode (failsafe configuration):
```yaml
    dcf_overlay:
      "2028sub1": "0x01" # UseMarkers (0 = disable marker detection, 1 = enable marker detection), UINT8, DataType=0x0005, defaultvalue=0
      "2028sub2": "0x02" # MarkerStyle (0 = disable marker detection, 1 = standard mode, 2 = extended mode), UINT8, DataType=0x0005, defaultvalue=0
      "2028sub3": "0x01" # FailSafeMode (0 = disabled, 1 = enabled), UINT8, DataType=0x0005, defaultvalue=0
```

## yaml configuration for OLS

The configuration of an OLS device is pretty much the same to MLS, just with a different PDO mapping and a different eds-file. Therefore, the object list specified by `publish`
and by `dcf_overlay` vary and the eds-file "SICK_OLS20.eds" is used. Otherwise, the same configuration apply.

This is the default configuration for a OLS10 device in file ols/sick_line_guidance_ols10.yaml:
```yaml
bus:
  device: can0
  driver_plugin: can::SocketCANInterface
  master_allocator: canopen::SimpleMaster::Allocator
sync:
  interval_ms: 10
  overflow: 0
nodes:
  node1:
    id: 0x0A # CAN-Node-ID of can device, default: Node-ID 10=0x0A for OLS and MLS
    eds_pkg: sick_line_guidance # package name for relative path
    eds_file: SICK_OLS10_CO.eds    # path to EDS/DCF file
    publish: ["1001","1018sub1","1018sub4","2021sub1!","2021sub2!","2021sub3!","2021sub4!","2021sub5!","2021sub6!","2021sub7!","2021sub8!"]
    # OLS10: 1001 = error register, 1018sub1 = VendorID, 1018sub4 = SerialNumber, TPDOs 0x2021sub1 to 0x2021sub8
    
    # sick_line_guidance configuration of this node:
    sick_device_family: "OLS10"            # can devices currently supported: "OLS10", "OLS20" or "MLS"
    sick_topic: "ols"                      # OLS_Measurement messages are published in topic "/ols"
    sick_frame_id: "ols_measurement_frame" # OLS_Measurement messages are published with frame_id "ols_measurement_frame"

    # device configuration of writable parameter by dcf_overlay: "objectindex": "parametervalue"
    # example: "2001sub5": "0x01" # Object 2001sub5 (sensorFlipped, defaultvalue 0x00) will be configured with value 0x01
    # dcf_overlay:
    #   "2001sub5": "0x01"  # sensorFlipped, UINT8, DataType=0x0005, defaultvalue=0
    #   "2002sub1": "0.015" # Typ. Width, FLOAT32, DataType=0x0008
    #   "2002sub2": "0.001" # Min. Width, FLOAT32, DataType=0x0008
    #   "2002sub3": "0.050" # Max. Width, FLOAT32, DataType=0x0008
```

This is the default configuration for a OLS20 device in file ols/sick_line_guidance_ols20.yaml:
```yaml
bus:
  device: can0
  driver_plugin: can::SocketCANInterface
  master_allocator: canopen::SimpleMaster::Allocator
sync:
  interval_ms: 10
  overflow: 0
nodes:
  node1:
    id: 0x0A # CAN-Node-ID of can device, default: Node-ID 10=0x0A for OLS and MLS
    eds_pkg: sick_line_guidance # package name for relative path
    eds_file: SICK_OLS20_CO.eds    # path to EDS/DCF file
    publish: ["1001","1018sub1","1018sub4","2021sub1!","2021sub2!","2021sub3!","2021sub4!","2021sub5!","2021sub6!","2021sub7!","2021sub8!"]
    # OLS20: 1001 = error register, 1018sub1 = VendorID, 1018sub4 = SerialNumber, TPDOs 0x2021sub1 to 0x2021sub8
    
    # sick_line_guidance configuration of this node:
    sick_device_family: "OLS20"            # can devices currently supported: "OLS10", "OLS20" or "MLS"
    sick_topic: "ols"                      # OLS_Measurement messages are published in topic "/ols"
    sick_frame_id: "ols_measurement_frame" # OLS_Measurement messages are published with frame_id "ols_measurement_frame"

    # device configuration of writable parameter by dcf_overlay: "objectindex": "parametervalue"
    # example: "2001sub5": "0x01" # Object 2001sub5 (sensorFlipped, defaultvalue 0x00) will be configured with value 0x01
    # dcf_overlay:
    #   "2001sub5": "0x01"  # sensorFlipped, UINT8, DataType=0x0005, defaultvalue=0
    #   "2002sub1": "0.015" # Typ. Width, FLOAT32, DataType=0x0008
    #   "2002sub2": "0.001" # Min. Width, FLOAT32, DataType=0x0008
    #   "2002sub3": "0.050" # Max. Width, FLOAT32, DataType=0x0008
```

The following table describes these settings in detail:

| parameter name | default value | details |
| --- | --- | --- |
| bus / device | can0 | System name of can interface: net device driver for can hardware (f.e. pcan adapter) are installed to a net device named "can0". Check by system command "ifconfig -a"  |
| bus / driver_plugin | can:: SocketCANInterface | Implementation of interface SocketCAN, currently only can::SocketCANInterface is supported. |
| bus / master_allocator | canopen:: SimpleMaster:: Allocator | Implementation of CAN master (network manager), currently only canopen::SimpleMaster::Allocator is supported. |
| sync / interval_ms | 10 | Timeinterval for can sync messages in milliseconds (CAN master will send a nmt sync message each 10 ms) |
| sync / overflow | 0 | Length of can sync message (0: sync message does not contain any data byte) |
| nodes | node1 | Name of the first can node configured. If you're running more than 1 can device (multiple MLS or OLS devices), you can configure each device by providing a named section for each node. |
| **nodes / node1 / id** | **0x0A** | **CAN-Node-ID of can device (default: Node-ID 10=0x0A for OLS and MLS). Change this id, if your OLS uses a different CAN-ID. :exclamation:** |
| nodes / node1 / eds_pkg | sick_line_guidance | Name of sick_line_guidance ros package. Required to resolve filenames and should not be modified. |
| nodes / node1 / eds_file | SICK_OLS20.eds | Electronic datasheet for your OLS device  |
| nodes / node1 / publish | ["1001", "1018sub1", "1018sub4", "2021sub1!", "2021sub2!", "2021sub3!", "2021sub4!", "2021sub5!", "2021sub6!", "2021sub7!", "2021sub8!"] | List of published objects in the object dictionary of a can device. These objects are both published on ros topic <nodename>_<objectindex> and required by driver internal callbacks to handle PDO messages. Do not remove any PDO mapped objects from this list! |
| nodes / node1 / sick_device_family | "OLS20" | Informs the ros driver that this can device is an OLS20. Currently supported values are "OLS10", "OLS20" or "MLS". |
| nodes / node1 / sick_topic | "ols" | Name of the ros topic for publishing OLS measurement messages. If modified, the same topic must be set in file sick_line_guidance.launch for the sick_line_guidance_cloud_publisher node (param name="ols_topic_publish" type="str" value="ols"). Otherwise, measurement messges won't be transformed to PointCloud2 messsages. |
| nodes / node1 / sick_frame_id | "ols_measurement_frame" | Ros frame id of the OLS measurement messages. |
| **# nodes / node1 / dcf_overlay** |  | **Configuration of device parameter. To set an object in the object dictionary of a can device to a specific value at startup, append dcf_overlay entries with "objectindex": "parametervalue". This section is intensionally left empty to use default values. See operation manual for possible settings. :exclamation:** |

Device specific settings can be configured in section "nodes/node1/dcf_overlay". Example to set object 2001sub5 (sensorFlipped):
```yaml
    dcf_overlay:
      "2001sub5": "0x01" # sensorFlipped, UINT8, DataType=0x0005, defaultvalue=0
```

## Summary of the yaml configuration for MLS and OLS devices

Set the CAN-ID of your device with `id: 0x0A # CAN-Node-ID of MLS (default: 0x0A)` and
configure device specific settings in section `dcf_overlay` by appending a line `"objectindex": "parametervalue"` for each parameter.

## yaml configuration for multiple can devices

If you want to use multiple can devices (MLS or OLS) on one system, you have to append multiple nodes in your yaml-file. Each node must contain the configuration of
one can device as shown above. Just make sure to use different node names for each device:
```yaml
nodes:
  node1:
    id: ... # CAN-ID for the 1. device
  node2:
    id: ... # CAN-ID for the 2. device
```

If you configure different measurement topics for each can device (f.e. `sick_topic: "ols20-1"` for the first OLS20 and `sick_topic: "ols20-1"` for the second OLS20 device),
you have to start multiple sick_line_guidance_cloud_publisher nodes, too. Otherwise, measurement messges from this device can't be transformed to PointCloud2 messsages.
Append multiple sick_line_guidance_cloud_publisher nodes in file sick_line_guidance.launch, each sick_line_guidance_cloud_publisher node configured with the corresponding 
ros topic in parameter `"mls_topic_publish"` resp. `"ols_topic_publish"`.

## Read and write parameter during runtime

You can read and write to the object dictionary of a can device on runtime, too, by using the ros services implemented by the canopen driver.

To read a value from the object dictionary, you can run
```bash
rosservice call /driver/get_object "{node: '<nodename>', object: '<objectindex>', cached: <true|false>}"
```
in your terminal. Example to read object 2001sub5 (sensorFlipped) from node1:
```bash
rosservice call /driver/get_object "{node: 'node1', object: '2001sub5', cached: false}"
```
This command results in a can upload command using a SDO with object 2001sub5. The SDO response will be converted and displayed if the command succeded.

To write a value to the object dictionary, you can run
```bash
rosservice call /driver/get_object "{node: '<nodename>', object: '<objectindex>', value: '<parametervalue>', cached: <true|false>}"
```
in the terminal. Example to write value 0x01 to object 2001sub5 (sensorFlipped):
```bash
rosservice call /driver/set_object "{node: 'node1', object: '2001sub5', value: '0x01', cached: false}"
```
This command results in a can upload command using a SDO with object 2001sub5.
