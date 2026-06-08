import numpy as np
import matplotlib.pyplot as plt

"""
Wartośc zadan to prędkość obrotowa podana w jednotsce rpm - rotations per minute

Podajemy tą wartość na regulator PID który zamienia ją na odpowiednie wypełnienie

Obiekt sterowany to połaczenie motor driver i silnika, zamienia wypelnienia na odpowiednia wyjściową predkość obrotwą

Poprzrz sprzężenie zwrotne wyjscia z wejściem uzyskujemy uchyb podawany na regulator PID
"""

#obiekt
Vmax = 60 #[rpm]
pwmDuty = 255.0 #max wypelnienie
Ko = Vmax/pwmDuty
To = 0.1 #[s]

#controler
Kc = 12
Ti = 0.011
Td = 0.01
alfa = 0.0 #wpsolcznniki filatr czlonu D

#sim paremeters
dT = 0.0001
timeSpace = np.arange(0,3,dT)
R = 20
Y = np.zeros(len(timeSpace))
U = np.zeros(len(timeSpace))
Ui = 0.0
Ud = 0.0
errorPrev = 0

for i in range(len(timeSpace)):

    error = R - Y[i]

    Uk = Kc * error

    if Ti == 0: dUi = 0
    else: dUi = error * dT / Ti
    
    if i == 0 : Ud = 0
    elif alfa == 0:
        Ud = (Td * (error - errorPrev)) / dT
    elif Td != 0 and alfa != 0:
            dUd = ((Td * (error - errorPrev)/dT - Ud) / alfa) * dT
            Ud += dUd

    if 0 < (Ud + Uk + (Ui + dUi)) < 255:
        Ui += dUi

    errorPrev = error

    Ut = max(0,min(255,(Uk + Ui + Ud)))

    U[i] = Ut

    dY = (Ko*Ut - Y[i]) / To * dT

    if i < len(timeSpace)-1 : Y[i+1] = Y[i] + dY


plt.figure(figsize=(12,10))
plt.subplot(2,1,1)
plt.plot(timeSpace,Y[-len(timeSpace):])
plt.title(r"$y(t)$")
plt.xlabel('time [s]')
plt.ylabel(r'Prędkość [rpm]')
plt.grid()
plt.subplot(2,1,2)
plt.plot(timeSpace,U)
plt.title(r'$u(t)$')
plt.xlabel('time [s]')
plt.ylabel('PWM')
plt.grid()
plt.show()