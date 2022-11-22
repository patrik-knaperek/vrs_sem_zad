# **VRS Semestrálne zadanie: USART/CAN prevodník**

IMU (Inertial measurement unit) Xsens MTi-G-710 je združený snímač, ktorý pozostáva z akcelerometra, gyroskopu a GPS modulu. Poskytuje tak informácie o lineárnych a angulárnych zrýchleniach, orientácii a pozícii, ktoré zohrávajú kľúčovú úlohu pri určení jazdného stavu vozidla. Tieto údaje sú natívne posielané po USART zbernici cez USB rozhranie. Komunikácia medzi jednotlivými MCU v [študentskej formule SGT-FE22](https://sgteam.eu/) je zabezpečovaná prostredníctvom CAN zbernice. Hlavným cieľom tohto zadania je naprogramovať prevodník z USART na CAN zbernicu, ktorý zabezpečí odosielanie dát z IMU ostatným prvkom elektronického systému SGT-FE22.

<p align="left">
    <img src="./doc/VRS_semestralne_zadanie_schema.svg" width="900">
</p>

## **Ciele zadania**
* naprogramovať IMU Bridge MCU:
    - konverzia správ USART ➝ CAN
    - vyšetrovanie zaťaženia CAN zbernice – podiel dát prúdiacich z IMU na celkovom zaťažení CAN
* vytvoriť užívateľské prostredie prostredníctvom [STM32CubeMonitor](https://www.st.com/en/development-tools/stm32cubemonitor.html) a Node-RED<sup>®</sup> 
    - inšpekcia toku dát a zaťaženia CAN zbernice
    - konfigurácia parametrov komunikačných rozhraní CAN a USART – pre prípad napojenia zberníc s inými parametrami (napr. baud rate) 
* naprogramovať hardvérovú signalizáciu komunikácie pomocou LED diód

## **Referencie**
* [IMU Bridge schéma](./doc/IMU_Bridge_sheet.pdf)
* [MCU datasheet](./doc/stm32f105_datasheet.pdf)
* [IMU reference manual](./doc/MTi_familyreference_manual.pdf)
* [IMU user manual](./doc/MTi_usermanual.pdf)
* [IMU configuration tool manual](./doc/MT_Manager_user_manual.pdf)