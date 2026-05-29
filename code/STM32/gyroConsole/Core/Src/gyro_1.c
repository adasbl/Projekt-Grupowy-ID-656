/*
 * kalibracja.c
 *
 *  Created on: Jan 23, 2026
 *      Author: adamj
 */
#include "main.h"
#include <stdio.h>
#include "iks4a1_motion_sensors.h"
#include "custom_bus.h"


extern IKS4A1_MOTION_SENSOR_Axes_t gyr_axes;
extern int32_t gyro_blad_z;
extern float obecnyAngle;
extern uint32_t tick_ostatni;
extern float dt;
extern int8_t LSM6DSV16X_INSTANCE;

void gyro_kalibracja()
{
	printf("Kalibracja w toku... Nie ruszaj plytka!\r\n");

	int32_t suma_z = 0;
	int samples = 100;

	for (int i = 0; i < samples; i++)
	{
		IKS4A1_MOTION_SENSOR_GetAxes(LSM6DSV16X_INSTANCE, MOTION_GYRO, &gyr_axes);
		suma_z += gyr_axes.z;
		HAL_Delay(10);
	}

	gyro_blad_z = suma_z / samples;
	obecnyAngle = 0.0f;

	printf("Kalibracja zakonczona. Bias Z: %ld mdps\r\n", gyro_blad_z);

	// Czekamy chwilę, żeby puszczenie przycisku nie wywołało od razu kolejnych akcji
	HAL_Delay(500);
}

// Funkcja 2: Obliczanie kąta (wywoływana co 100ms)
void gyro_kat()
{
	// Pobierz aktualny czas, aby obliczyć dokładne dt (powinno być ok. 0.1s)
	uint32_t tick_now = HAL_GetTick();
	float dt = (tick_now - tick_ostatni) / 1000.0f;

	if(IKS4A1_MOTION_SENSOR_GetAxes(LSM6DSV16X_INSTANCE, MOTION_GYRO, &gyr_axes) == 0)
	{
		// Odejmij błąd statyczny (kalibrację)
		float gyro_z_poprawiona = (float)(gyr_axes.z - gyro_blad_z);

		// Filtracja szumów (Deadzone) - jeśli ruch jest bardzo mały, ignoruj
		if (gyro_z_poprawiona > -1000.0f && gyro_z_poprawiona < 1000.0f) {
			gyro_z_poprawiona = 0.0f;
		}

		// Całkowanie: Kąt += (Prędkość * Czas)
		// Dzielimy przez 1000.0f, bo żyroskop zwraca mdps (mili-stopnie na sekundę), a chcemy stopnie
		obecnyAngle += (gyro_z_poprawiona / 1000.0f) * dt;

		printf("Z: %5ld | Kat: %.2f st.\r\n", gyr_axes.z, obecnyAngle);
		printf("----------------- \r\n");
	}
}
