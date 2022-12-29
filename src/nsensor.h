/*FULL_OPTION is set as type, when you want the device to have
static ai detection, ambient and motion*/
#define FULL_OPTION_TYPE         0x24  

/*MOTION_AMBIENT is set as type, when you want the device to have
 motion and ambient only*/
#define MOTION_AMBIENT_TYPE      0x25

/*MOTION is set as type, when you want the device to have motion only */
#define MOTION_ONLY_TYPE         0x26

/*Sensor Firmware Version App - Main version */
#define V_APP               0x1

/*Sensor Firmware Version Major Number **/
#define V_MAJOR                     0x0

/*Sensor Firmware Version Minor Number*/
#define V_MINOR                     0x18

#define BIT_RATE                    2000000

/*(csi2) SDA Line for Camera*/
#define GPIO_04                     4

/*(csi2) SCL Line for Camera*/
#define GPIO_14                     14

/*Ncontroller issues this command for getting the MAC address of nsensor*/
#define SYNC_MAC_CMD                0x42

/*Ncontroller issues this command to get the ambient value from the sensor*/
#define READ_AMBIENT_CMD            0x4C

/*Ncontroller command to query the type of the sensor
nsensor are 3 types
1) Full option sensor
2) Motion ambient 
3) Only ambient */
#define GET_SENSOR_TYPE_CMD         0x5E

/*Get the current version of sensor firmware*/
#define GET_SENSOR_FIRMWARE_VERSION  0x5D

/*Ncontroller issue this command to reset the sensor*/
#define CHANGE_RESOLUTION_CMD        0x7F

#define GET_RAW_CAMERA_IMAGE         0x55


