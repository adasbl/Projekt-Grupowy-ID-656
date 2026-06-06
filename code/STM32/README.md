# Opis programów

## Programy głowne do sterownia robotem
### ver1
Sterowanie ruchem i skręcaniem z wykorzystaniem nowych silników i enkoderów (timery w trybie enkodera) – zmiana kierunku jazdy przyciskiem (na przerwaniach). Dla jazdy prosto regulator PID z Anti-Windup oraz stopniowe rozpędzanie robota (soft-start).

### ver2
Sterowanie ruchem z wykorzystaniem niezależnych regulatorów PI dla obu silników wyposażonych w kompensację strefy nieczułości (deadband) oraz Anti-Windup, na podstawie docelowej prędkości (wyrażonej w impulsach enkodera). Zmiana kierunku jazdy (prosto, obrót w miejscu, tył) realizowana przyciskiem na przerwaniach, obsługiwana przez algorytm z kompensacją strefy nieczułości (deadband). 

### ver3
Algorytm jazdy wykorzystujący zadane wartości prędkości kątowej i liniowej (wyrażonych w m/s i rad/s)) - zgodnie z ramką `Twist` odbieraną przerwaniami przez UART. Prędkości automatycznie przeliczane na podstawie zaimplementowanych parametrów fizycznych platformy. Regulatory PI z kompensacją strefy nieczułości (deadband) oraz Anti-Windup.

&nbsp;&nbsp;&nbsp;

## Programy testowe i diagnostyczne
### diodeButtonTest
Program testujący diody oraz przyciski na wbudowanym PCB.

### engineTest
Program uruchamiający koła z konkretnie określoną prędkością  
(bez włączania enkoderów i komunikacji UART).

### enkoderTest
Program inicjujący enkodery i zliczający ich wartości  
(do użycia w trybie debugowania).

### uartTest
Program testujący komunikację UART pomiędzy Jetson Nano a STM32 (uart3). Dane są printowane do konsoli (uart2)  
(do użycia z nucleo, płytka wbudowana nie ma wyprowadzone uart2).

### gyroConsole
Program do obsługi i testowania żyroskopu przez konsolę.

### sterowanieESP
Jazda w zadanym kierunku poprzez ESP32 z wykorzystaniem silników i enkoderów.