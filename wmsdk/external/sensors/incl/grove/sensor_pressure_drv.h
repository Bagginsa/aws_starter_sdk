/*
 *  Copyright (C) 2015-2016, Marvell International Ltd.
 *  All Rights Reserved.
 */

/*
 * Custom Sensor Driver header file
 */

#ifndef __SENSOR_PRESSURE_DRV_H__
#define __SENSOR_PRESSURE_DRV_H__

#define BMP085_ADDRESS 0x77

struct Barometer {
//    public:
    void (*init)(void);
    long PressureCompensate;
    float (*bmp085GetTemperature)(unsigned short ut);
    long (*bmp085GetPressure)(unsigned long up);
    float (*calcAltitude)(float pressure);
    unsigned short (*bmp085ReadUT)(void);
    unsigned long (*bmp085ReadUP)(void);

//    private:

    short ac1;
    short ac2;
    short ac3;
    unsigned short ac4;
    unsigned short ac5;
    unsigned short ac6;
    short b1;
    short b2;
    short mb;
    short mc;
    short md;
    char (*bmp085Read)(unsigned char address);
    short (*bmp085ReadInt)(unsigned char address);
    void (*writeRegister)(short deviceAddress, uint8_t address, uint8_t val);
    short (*readRegister)(short deviceAddress, uint8_t address);
    short ut;
    short up;
};

int pressure_sensor_event_register(void);

/* You can declare customer sensor specific declaratio here */

#endif /* __SENSOR_PRESSURE_DRV_H__ */
