#!/usr/bin/env python
# -*- coding: utf-8 -*-

# /!\ Ligne au dessus très importante
# Nécessaire pour faire tourner ros sous python
import smbus
import numpy as np
import rospy
from geometry_msgs.msg import AccelStamped

class IMU():
    """Driver for the MPU-9250 based on the revision 1.4 of the 9/9/2013
    For every register "_H" and "_L" contains the High and Low bytes.
    0x00 is the reset value for all registers EXCEPT :
        - Register 107 (0x01) Power Management 1
        - Register 117 (0x71) WHO_AM_I"""

    # Values from manufacturing tests
    SELF_TEST_X_GYRO    = 0x00 # r/w
    SELF_TEST_Y_GYRO    = 0x01 # r/w
    SELF_TEST_Z_GYRO    = 0x02 # r/w
    SELF_TEST_X_ACCEL   = 0x0D # r/w
    SELF_TEST_Y_ACCEL   = 0x0E # r/w
    SELF_TEST_Z_ACCEL   = 0x0F # r/w

    # Used to remove DC bias from the gyro sensor data output for X, Y and Z axes
    # The values in these registers are subtracted from the gyro sensor values before going into
    # the sensor registers.
    XG_OFFSET_H         = 0x13 # r/w
    XG_OFFSET_L         = 0x14 # r/w
    YG_OFFSET_H         = 0x15 # r/w
    YG_OFFSET_L         = 0x16 # r/w
    ZG_OFFSET_H         = 0x17 # r/w
    ZG_OFFSET_L         = 0x18 # r/w

    SMPLRT_DIV          = 0x19 # r/w
    CONFIG              = 0x1A # r/w
    GYRO_CONFIG         = 0x1B # r/w
    ACCEL_CONFIG        = 0x1C # r/w
    ACCEL_CONFIG2       = 0x1D # r/w
    LP_ACCEL_ODR        = 0x1E # r/w
    WOM_THR             = 0x1F # r/w
    FIFO_EN             = 0x23 # r/w
    I2C_MST_CTRL        = 0x24 # r/w
    I2C_SLV0_ADDR       = 0x25 # r/w
    I2C_SLV0_REG        = 0x26 # r/w
    I2C_SLV0_CTRL       = 0x27 # r/w
    I2C_SLV1_ADDR       = 0x28 # r/w
    I2C_SLV1_REG        = 0x29 # r/w
    I2C_SLV1_CTRL       = 0x2A # r/w
    I2C_SLV2_ADDR       = 0x2B # r/w
    I2C_SLV2_REG        = 0x2C # r/w
    I2C_SLV2_CTRL       = 0x2D # r/w
    I2C_SLV3_ADDR       = 0x2E # r/w
    I2C_SLV3_REG        = 0x2F # r/w
    I2C_SLV3_CTRL       = 0x30 # r/w
    I2C_SLV4_ADDR       = 0x31 # r/w
    I2C_SLV4_REG        = 0x32 # r/w
    I2C_SLV4_DO         = 0x33 # r/w
    I2C_SLV4_CTRL       = 0x34 # r/w
    I2C_SLV4_DI         = 0x35 # r
    I2C_MST_STATUS      = 0x36 # r
    INT_PIN_CFG         = 0x37 # r/w
    INT_ENABLE          = 0x38 # r/w
    INT_STATUS          = 0x3A # r
    ACCEL_XOUT_H        = 0x3B # r
    ACCEL_XOUT_L        = 0x3C # r
    ACCEL_YOUT_H        = 0x3D # r
    ACCEL_YOUT_L        = 0x3E # r
    ACCEL_ZOUT_H        = 0x3F # r
    ACCEL_ZOUT_L        = 0x40 # r
    TEMP_OUT_H          = 0x41 # r
    TEMP_OUT_L          = 0x42 # r
    GYRO_XOUT_H         = 0x43 # r
    GYRO_XOUT_L         = 0x44 # r
    GYRO_YOUT_H         = 0x45 # r
    GYRO_YOUT_L         = 0x46 # r
    GYRO_ZOUT_H         = 0x47 # r
    GYRO_ZOUT_L         = 0x48 # r
    EXT_SENS_DATA_00    = 0x49 # r
    EXT_SENS_DATA_01    = 0x4A # r
    EXT_SENS_DATA_02    = 0x4B # r
    EXT_SENS_DATA_03    = 0x4C # r
    EXT_SENS_DATA_04    = 0x4D # r
    EXT_SENS_DATA_05    = 0x4E # r
    EXT_SENS_DATA_06    = 0x4F # r
    EXT_SENS_DATA_07    = 0x50 # r
    EXT_SENS_DATA_08    = 0x51 # r
    EXT_SENS_DATA_09    = 0x52 # r
    EXT_SENS_DATA_10    = 0x53 # r
    EXT_SENS_DATA_11    = 0x54 # r
    EXT_SENS_DATA_12    = 0x55 # r
    EXT_SENS_DATA_13    = 0x56 # r
    EXT_SENS_DATA_14    = 0x57 # r
    EXT_SENS_DATA_15    = 0x58 # r
    EXT_SENS_DATA_16    = 0x59 # r
    EXT_SENS_DATA_17    = 0x5A # r
    EXT_SENS_DATA_18    = 0x5B # r
    EXT_SENS_DATA_19    = 0x5C # r
    EXT_SENS_DATA_20    = 0x5D # r
    EXT_SENS_DATA_21    = 0x5E # r
    EXT_SENS_DATA_22    = 0x5F # r
    EXT_SENS_DATA_23    = 0x60 # r
    I2C_SLV0_DO         = 0x63 # r/w
    I2C_SLV1_DO         = 0x64 # r/w
    I2C_SLV2_DO         = 0x65 # r/w
    I2C_SLV3_DO         = 0x66 # r/w
    I2C_MST_DELAY_CTRL  = 0x67 # r/w
    SIGNAL_PATH_RESET   = 0x68 # r/w
    MOT_DETECT_CTRL     = 0x69 # r/w
    USER_CTRL           = 0x6A # r/w
    PWR_MGMT_1          = 0x6B # r/w
    PWR_MGMT_2          = 0x6C # r/w
    FIFO_COUNTH         = 0x72 # r/w
    FIFO_COUNTL         = 0x73 # r/w
    FIFO_R_W            = 0x74 # r/w
    WHO_AM_I            = 0x75 # r
    XA_OFFSET_H         = 0x77 # r/w
    XA_OFFSET_L         = 0x78 # r/w
    YA_OFFSET_H         = 0x7A # r/w
    YA_OFFSET_L         = 0x7B # r/w
    ZA_OFFSET_H         = 0x7D # r/w
    ZA_OFFSET_L         = 0x7E # r/w

    def __init__(self, ADDRESS = IMU.WHO_AM_I, bus = 0):
        self.address = ADDRESS
        self.bus = bus

        CONFIG_value        = 0b00000001 # RESERVED / FIFO_MODE = 0 / EXT_SYNC_SET = 000 / DLPF_CFG = 000 >> FCHOICE = 00 or 01
        GYRO_CONFIG_value   = 0b00000010 # XGYRO_Cten = 0 / YGYRO_Cten = 0 / ZGYRO_Cten = 0 / GYRO_FS_SEL = 00 / RESERVED / Fchoice_b = 11 or 10
        ACCEL_CONFIG_value  = 0b00001000 # ax_st_en = 0 / ay_st_en = 0 / az_st_en = 0 / ACCEL_FS_SEL = 01 / RESERVED
        ACCEL_CONFIG2_value = 0b00000000 # RESERVED / RESERVED / accel_fchoice_b = 0 / A_DLPFCFG = 000
        WOM_THR = 0b00000000
        config_CTRL8 = 0b10100101
        config_CTRL9 = 0b00111000
        config_CTRL10 = 0b00111101

        # self.bus.write_byte_data(self.address,0x10,config_CTRL1)
        # self.bus.write_byte_data(self.address,0x11,config_CTRL2)
        # self.bus.write_byte_data(self.address,0x14,config_CTRL5)
        # self.bus.write_byte_data(self.address,0x15,config_CTRL6)
        # self.bus.write_byte_data(self.address,0x16,config_CTRL7)
        # self.bus.write_byte_data(self.address,0x17,config_CTRL8)
        # self.bus.write_byte_data(self.address,0x18,config_CTRL9)
        # self.bus.write_byte_data(self.address,0x19,config_CTRL10)

    def read_acc(self):
        # Xl_XL, Xh_XL, Yl_XL, Yh_XL, Zl_XL, Zh_XL = self.bus.read_i2c_block_data(self.address, self.OUTX_L_XL, 6)
        X_XL = np.int16(Xh_XL*0x100+Xl_XL)/100
        Y_XL = np.int16(Yh_XL*0x100+Yl_XL)/100
        Z_XL = np.int16(Zh_XL*0x100+Zl_XL)/100

        return X_XL*0.448, Y_XL*0.488, Z_XL*0.488

    def read_gyro(self):
        # Xl_G, Xh_G, Yl_G, Yh_G, Zl_G, Zh_G = self.bus.read_i2c_block_data(self.address, self.OUTX_L_G, 6)
        X_G = np.int16(Xh_G*0x100+Xl_G) 
        Y_G = np.int16(Yh_G*0x100+Yl_G)
        Z_G = np.int16(Zh_G*0x100+Zl_G)

        return X_G*8.75, Y_G*8.75, Zl_G*8.75

    def read_All(self):
        # X_G, Y_G, Z_G = self.read_gyro()
        # X_ACC, Y_ACC, Z_ACC = self.read_acc()
        X_G, Y_G, Z_G = 0, 0, 0
        X_ACC, Y_ACC, Z_ACC = 0, 0, 0

        return X_ACC, Y_ACC ,Z_ACC, X_G, Y_G, Z_G

def node(f = 25):
    # Init
    rospy.init_node('IMU', anonymous=True)
    pub = rospy.Publisher('IMU_data', AccelStamped, queue_size=10)
    msg = AccelStamped()

    rate = rospy.Rate(f)
    # Possibilité faire un fichier externe à lire pour avoir les fréquences

    # Boucle ROS
    while not rospy.is_shutdown():
        
        # Lecture IMU
        x_acc, y_acc, z_acc, x_g, y_g, z_g = imu.read_All()
        
        # Update msg
        msg.header.stamp = rospy.Time.now()
        msg.accel.linear.x = x_acc
        msg.accel.linear.y = y_acc
        msg.accel.linear.z = z_acc
        msg.accel.angular.x = x_g
        msg.accel.angular.y = y_g
        msg.accel.angular.z = z_g
        
        # Publish

        # rospy.loginfo("Sent : x_acc = " + str(x_acc) + ", y_acc = " + str(y_acc) + ", z_acc = " + str(z_acc))
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    imu = IMU()
    try:
        node()
    except rospy.ROSInterruptException:
        pass
