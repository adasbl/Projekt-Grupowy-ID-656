# Opis programów

## sterowanieESP
Jazda w zadanym kierunku poprzez ESP32 z wykorzystaniem silników i enkoderów.

## diodeButtonTest
Program testujący diody oraz przyciski na wbudowanym PCB.

## engineTest
Program uruchamiający koła z konkretnie określoną prędkością  
(bez włączania enkoderów i komunikacji UART).

## enkoderTest
Program inicjujący enkodery i zliczający ich wartości  
(do użycia w trybie debugowania).

## uartTest
Program testujący komunikację UART pomiędzy Jetson Nano a STM32 (uart3). Dane są printowane do konsoli (uart2)  
(do użycia z nucleo, płytka wbudowana nie ma wyprowadzone uart2).

## gyroConsole
Program do obsługi i testowania żyroskopu przez konsolę.

## ver1
Sterowanie ruchem i skręcaniem z wykorzystaniem nowych silników i enkoderów (timery w trybie enkodera) – zmiana kierunku jazdy przyciskiem (na przerwaniach). Dla jazdy prosto regulator PID oraz stopniowe rozpędzanie robota (soft-start).

## ver2
Algorytm jazdy wykorzystujący zadane wartości prędkości kątowej i liniowej, zgodnie z ramką `Twist` odbieraną przerwaniami przez UART.