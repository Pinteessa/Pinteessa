/*
 * datacollect.c
 *
 *  Created on: Nov 6, 2024
 *      Author: linuxlite
 */
#include <stdio.h>
#include "sensors/mpu9250.h"
#include <ti/drivers/I2C.h>

// Datan keruuseen liittyvä funktio
void startDataCollection(I2C_Handle i2c) {
    float ax, ay, az, gx, gy, gz;

    while (1) {
        // Kerätään dataa sensorilta
        mpu9250_get_data(i2c, &ax, &ay, &az, &gx, &gy, &gz);

        // Tulostetaan kerätty data
        printf("Acc: %.4f %.4f %.4f, Gyro: %.4f %.4f %.4f\n", ax, ay, az, gx, gy, gz);
    }
}
