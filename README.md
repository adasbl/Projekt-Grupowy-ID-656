# Dron kołowy do autonomicznego mapowania terenu

Projekt autonomicznego robota mobilnego opartego na układzie różnicowym (differential drive). Projekt został zrealizowany we współpracy z [Zakładem Automatyki i Urządzeń Pomiarowych AREX Sp. z o.o](https://www.wbgroup.pl/arex/).

<div align="center">
    <img src="img/photo1.jpeg" width="60%">
</div>

## Struktura Repozytorium

* `3Dmodels/` - Modele 3D:
  * `other/` - Pozostałe modele 3D.
  * `platform/` - Modele 3D platformy robota.
* `code/` - Kod źródłowy i symulacje:
  * `ESP32/` - Oprogramowanie dla mikrokontrolerów ESP32.
  * `JETSON/` - Oprogramowanie dla jednostki nadrzędnej Nvidia Jetson.
  * `ros_simulation/` - Pliki symulacyjne dla środowiska ROS.
  * `simulation/` - Środowisko symulacyjne.
  * `STM32/` - Oprogramowanie dla mikrokontrolera STM32.
* `datasheets/` - Noty katalogowe wykorzystanych podzespołów.
* `docs/` - Dokumentacja techniczna modułów (np. lista komponentów, bilans mocy).
* `docsPG/` - Akademicka dokumentacja projektu.
* `images/` - Zdjęcia związane z projektem.
* `pcb/` (Submoduł) - Projekt autorskiej płytki sterownika wykonawczego.
* `robot-setup/` (Submoduł) - Skrypty i konfiguracja początkowa robota.

## Powiązane Repozytoria i Zewnętrzne Odnośniki

* [Oprogramowanie i instrukcja do obsługi robota (ROS2)](https://github.com/matrix1798/robot_sofware.git)
* [Repozytorium płytki sterownika wykonawczego (KiCad)](https://github.com/adasbl/main-controller-pcb.git)

## Zespół Projektowy

* [Adam Błażejewski](https://github.com/adasbl)
* [Mateusz Kuczerowski](https://github.com/matrix1798)
* [Maciej Domeradzki](https://github.com/TofTo23)
* [Krzysztof Toczyński](https://github.com/rolomixedmixed)