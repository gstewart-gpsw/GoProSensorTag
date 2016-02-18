# GoProSensorTag
SensorTag with GoPro Compatible GATT table


#GoPro Sensortag Build Insructions

1. Download TI Stack
BLE-STACK-CC25X: Version 1.4.0
http://www.ti.com/tool/BLE-STACK-ARCHIVE
(note - 1.4.1 will require additional porting)

2. Download TI 8051 Compiler from IAR.com
Select compiler for CC2541F128 for 8051

3. Download Projects.zip
Extract to ..\BLE-CC254x-1.4.0\

4. Open project
IAR Compiler
File/Open/Open Workspace
C:\..\BLE-CC254x-1.4.0\Projects\ble\SensorTag - GoPro\CC2541DB\SensorTag.eww

5. Build Project
Project>Rebuild All

6. Download to Sensortag  using CC Debugger
Project > Download and Debug

#Operation
Side Button
..Start/Stop Advertising when not connected
..Start/Stop Metadata when connected

Right Top Button
..Start/Stop shutter

Left Top Button
..Change mode

#Program Modification

..GOPRO_PACKET_RATE_MS  1000     // how often we will send metadata packets

..GOPRO_PACKET_COUNT       1     // number of packets to send

Modify Metadata Payload
See GoPro BLE SDK guide for information on 4CC formats


