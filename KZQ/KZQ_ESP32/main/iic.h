#ifndef _IIC_H
#define _IIC_H
void i2c_test_task(void *arg);
enum{
    dht11=1,
    zp13,
    zp15,
    zh06,
    sgp30,
    blux30,
    power,
    water
};

#endif