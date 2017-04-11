EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:nrf24l01+
LIBS:lt1944
LIBS:footswitch-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "MIDI footswitch"
Date "2017-04-12"
Rev "1.0"
Comp "Jaakko Salo / jaakkos@gmail.com"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA328P-A IC?
U 1 1 58EBB3D2
P 2450 2250
F 0 "IC?" H 1700 3500 50  0000 L BNN
F 1 "ATMEGA328P-A" H 2850 850 50  0000 L BNN
F 2 "TQFP32" H 2450 2250 50  0000 C CIN
F 3 "" H 2450 2250 50  0000 C CNN
	1    2450 2250
	1    0    0    -1  
$EndComp
Text GLabel 1325 925  1    39   Input ~ 0
3V3
$Comp
L Earth #PWR?
U 1 1 58EBB46B
P 1325 3625
F 0 "#PWR?" H 1325 3375 50  0001 C CNN
F 1 "Earth" H 1325 3475 50  0001 C CNN
F 2 "" H 1325 3625 50  0000 C CNN
F 3 "" H 1325 3625 50  0000 C CNN
	1    1325 3625
	1    0    0    -1  
$EndComp
$Comp
L DC04-11 AFF?
U 1 1 58EBB770
P 9025 4275
F 0 "AFF?" H 9025 4775 50  0000 C CNN
F 1 "DC04-11" H 9025 3825 50  0000 C CNN
F 2 "" H 9025 4275 50  0000 C CNN
F 3 "" H 9025 4275 50  0000 C CNN
	1    9025 4275
	1    0    0    -1  
$EndComp
$Comp
L BSS138 Q?
U 1 1 58EBBDC9
P 7300 4225
F 0 "Q?" H 7150 4375 50  0000 L CNN
F 1 "BSS214NWH6327XTSA1" V 7525 3625 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 7500 4150 50  0001 L CIN
F 3 "" H 7300 4225 50  0000 L CNN
	1    7300 4225
	1    0    0    -1  
$EndComp
$Comp
L BSS138 Q?
U 1 1 58EBBE8D
P 6600 4225
F 0 "Q?" H 6475 4400 50  0000 L CNN
F 1 "BSS214NWH6327XTSA1" V 6825 3625 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 6800 4150 50  0001 L CIN
F 3 "" H 6600 4225 50  0000 L CNN
	1    6600 4225
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 58EBC7CA
P 7400 4425
F 0 "#PWR?" H 7400 4175 50  0001 C CNN
F 1 "Earth" H 7400 4275 50  0001 C CNN
F 2 "" H 7400 4425 50  0000 C CNN
F 3 "" H 7400 4425 50  0000 C CNN
	1    7400 4425
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 58EBC7E8
P 6700 4425
F 0 "#PWR?" H 6700 4175 50  0001 C CNN
F 1 "Earth" H 6700 4275 50  0001 C CNN
F 2 "" H 6700 4425 50  0000 C CNN
F 3 "" H 6700 4425 50  0000 C CNN
	1    6700 4425
	1    0    0    -1  
$EndComp
Text GLabel 7100 4275 0    40   Input ~ 0
GPX
Text GLabel 6400 4275 0    40   Input ~ 0
GPX
Text GLabel 7725 5800 3    40   Input ~ 0
GPX
$Comp
L R_Small R?
U 1 1 58EBCC7E
P 7725 5700
F 0 "R?" V 7725 5675 24  0000 L CNN
F 1 "R_Small" V 7700 5775 24  0000 L CNN
F 2 "" H 7725 5700 50  0000 C CNN
F 3 "" H 7725 5700 50  0000 C CNN
	1    7725 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58EBCDE7
P 7800 5700
F 0 "R?" V 7800 5675 24  0000 L CNN
F 1 "R_Small" V 7775 5775 24  0000 L CNN
F 2 "" H 7800 5700 50  0000 C CNN
F 3 "" H 7800 5700 50  0000 C CNN
	1    7800 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58EBCE0F
P 7875 5700
F 0 "R?" V 7875 5675 24  0000 L CNN
F 1 "R_Small" V 7850 5775 24  0000 L CNN
F 2 "" H 7875 5700 50  0000 C CNN
F 3 "" H 7875 5700 50  0000 C CNN
	1    7875 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58EBCE36
P 7950 5700
F 0 "R?" V 7950 5675 24  0000 L CNN
F 1 "R_Small" V 7925 5775 24  0000 L CNN
F 2 "" H 7950 5700 50  0000 C CNN
F 3 "" H 7950 5700 50  0000 C CNN
	1    7950 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58EBCE66
P 8025 5700
F 0 "R?" V 8025 5675 24  0000 L CNN
F 1 "R_Small" V 8000 5775 24  0000 L CNN
F 2 "" H 8025 5700 50  0000 C CNN
F 3 "" H 8025 5700 50  0000 C CNN
	1    8025 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58EBCE93
P 8100 5700
F 0 "R?" V 8100 5675 24  0000 L CNN
F 1 "R_Small" V 8075 5775 24  0000 L CNN
F 2 "" H 8100 5700 50  0000 C CNN
F 3 "" H 8100 5700 50  0000 C CNN
	1    8100 5700
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58EBCFC3
P 8175 5700
F 0 "R?" V 8175 5675 24  0000 L CNN
F 1 "R_Small" V 8150 5775 24  0000 L CNN
F 2 "" H 8175 5700 50  0000 C CNN
F 3 "" H 8175 5700 50  0000 C CNN
	1    8175 5700
	1    0    0    -1  
$EndComp
Text GLabel 7800 5800 3    40   Input ~ 0
GPX
Text GLabel 7875 5800 3    40   Input ~ 0
GPX
Text GLabel 7950 5800 3    40   Input ~ 0
GPX
Text GLabel 8025 5800 3    40   Input ~ 0
GPX
Text GLabel 8100 5800 3    40   Input ~ 0
GPX
Text GLabel 8175 5800 3    40   Input ~ 0
GPX
Text GLabel 3875 1650 2    40   Input ~ 0
SCK
Text GLabel 3875 1450 2    40   Output ~ 0
MOSI
Text GLabel 3875 1550 2    40   Input ~ 0
MISO
Text GLabel 3900 2750 2    40   Input ~ 0
RXD
Text GLabel 3900 2850 2    40   Output ~ 0
TXD
$Comp
L C_Small C?
U 1 1 58EC20DD
P 1425 1925
F 0 "C?" V 1550 1875 50  0000 L CNN
F 1 "100nF" V 1300 1825 50  0000 L CNN
F 2 "" H 1425 1925 50  0000 C CNN
F 3 "" H 1425 1925 50  0000 C CNN
	1    1425 1925
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 58EC234C
P 1425 2025
F 0 "#PWR?" H 1425 1775 50  0001 C CNN
F 1 "Earth" H 1425 1875 50  0001 C CNN
F 2 "" H 1425 2025 50  0000 C CNN
F 3 "" H 1425 2025 50  0000 C CNN
	1    1425 2025
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P?
U 1 1 58EC2CAD
P 10225 1375
F 0 "P?" H 10225 1725 50  0000 C CNN
F 1 "Programming header" V 10325 1375 50  0000 C CNN
F 2 "" H 10225 1375 50  0000 C CNN
F 3 "" H 10225 1375 50  0000 C CNN
	1    10225 1375
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 58EC2D79
P 10025 1125
F 0 "#PWR?" H 10025 875 50  0001 C CNN
F 1 "Earth" H 10025 975 50  0001 C CNN
F 2 "" H 10025 1125 50  0000 C CNN
F 3 "" H 10025 1125 50  0000 C CNN
	1    10025 1125
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58EC2E45
P 10025 1225
F 0 "#PWR?" H 10025 975 50  0001 C CNN
F 1 "Earth" H 10025 1075 50  0001 C CNN
F 2 "" H 10025 1225 50  0000 C CNN
F 3 "" H 10025 1225 50  0000 C CNN
	1    10025 1225
	0    1    1    0   
$EndComp
Text GLabel 10025 1325 0    40   Output ~ 0
3V3
Text GLabel 10025 1425 0    40   Output ~ 0
RX
Text GLabel 10025 1525 0    40   Input ~ 0
TX
Text GLabel 10025 1625 0    40   Output ~ 0
DTR
$Comp
L nRF24L01+ U?
U 1 1 58ED73BF
P 6450 2250
F 0 "U?" H 6450 1950 50  0001 C CNN
F 1 "nRF24L01+" H 6450 2550 50  0000 C CNN
F 2 "MODULE" H 6450 2350 50  0001 C CNN
F 3 "DOCUMENTATION" H 6450 2200 50  0001 C CNN
	1    6450 2250
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED7961
P 4950 2275
F 0 "#PWR?" H 4950 2025 50  0001 C CNN
F 1 "Earth" H 4950 2125 50  0001 C CNN
F 2 "" H 4950 2275 50  0000 C CNN
F 3 "" H 4950 2275 50  0000 C CNN
	1    4950 2275
	1    0    0    -1  
$EndComp
Text GLabel 5575 1325 1    40   Input ~ 0
VCC
$Comp
L C_Small C?
U 1 1 58ED7C7A
P 5275 1775
F 0 "C?" H 5285 1845 50  0000 L CNN
F 1 "10uF" H 5285 1695 50  0000 L CNN
F 2 "" H 5275 1775 50  0000 C CNN
F 3 "" H 5275 1775 50  0000 C CNN
	1    5275 1775
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58ED7CF5
P 4950 1775
F 0 "C?" H 4960 1845 50  0000 L CNN
F 1 "100nF" H 4960 1695 50  0000 L CNN
F 2 "" H 4950 1775 50  0000 C CNN
F 3 "" H 4950 1775 50  0000 C CNN
	1    4950 1775
	1    0    0    -1  
$EndComp
Text GLabel 7200 2050 2    40   Input ~ 0
SCK
Text GLabel 7200 2200 2    40   Input ~ 0
MOSI
Text GLabel 7200 2350 2    40   Output ~ 0
MISO
NoConn ~ 7200 2500
Text GLabel 5700 2350 0    40   Input ~ 0
NRF_CE
Text GLabel 5700 2500 0    40   Input ~ 0
NRF_CSN
$Comp
L CONN_01X08 P?
U 1 1 58EDCA87
P 9650 1475
F 0 "P?" H 9650 1925 50  0000 C CNN
F 1 "Atmega top" V 9750 1475 50  0000 C CNN
F 2 "" H 9650 1475 50  0000 C CNN
F 3 "" H 9650 1475 50  0000 C CNN
	1    9650 1475
	1    0    0    -1  
$EndComp
Text GLabel 9450 1125 0    40   UnSpc ~ 0
PD2
Text GLabel 9450 1225 0    40   UnSpc ~ 0
PD1
Text GLabel 9450 1325 0    40   UnSpc ~ 0
PD0
Text GLabel 9450 1425 0    40   UnSpc ~ 0
PC6
Text GLabel 9450 1525 0    40   UnSpc ~ 0
PC5
Text GLabel 9450 1625 0    40   UnSpc ~ 0
PC4
Text GLabel 9450 1725 0    40   UnSpc ~ 0
PC3
Text GLabel 9450 1825 0    40   UnSpc ~ 0
PC2
$Comp
L CONN_01X08 P?
U 1 1 58EDD361
P 9025 1475
F 0 "P?" H 9025 1925 50  0000 C CNN
F 1 "Atmega left" V 9125 1475 50  0000 C CNN
F 2 "" H 9025 1475 50  0000 C CNN
F 3 "" H 9025 1475 50  0000 C CNN
	1    9025 1475
	1    0    0    -1  
$EndComp
Text GLabel 8825 1125 0    40   UnSpc ~ 0
PC1
Text GLabel 8825 1225 0    40   UnSpc ~ 0
PC0
Text GLabel 8825 1325 0    40   UnSpc ~ 0
ADC7
$Comp
L Earth #PWR?
U 1 1 58EDD62C
P 8825 1425
F 0 "#PWR?" H 8825 1175 50  0001 C CNN
F 1 "Earth" H 8825 1275 50  0001 C CNN
F 2 "" H 8825 1425 50  0000 C CNN
F 3 "" H 8825 1425 50  0000 C CNN
	1    8825 1425
	0    1    1    0   
$EndComp
Text GLabel 8825 1525 0    40   UnSpc ~ 0
AREF
Text GLabel 8825 1625 0    40   UnSpc ~ 0
ADC6
Text GLabel 8825 1725 0    40   UnSpc ~ 0
AVCC
Text GLabel 8825 1825 0    40   UnSpc ~ 0
PB5
$Comp
L CONN_01X08 P?
U 1 1 58EDD9D5
P 9025 2450
F 0 "P?" H 9025 2900 50  0000 C CNN
F 1 "Atmega bottom" V 9125 2450 50  0000 C CNN
F 2 "" H 9025 2450 50  0000 C CNN
F 3 "" H 9025 2450 50  0000 C CNN
	1    9025 2450
	1    0    0    -1  
$EndComp
Text GLabel 8825 2100 0    40   UnSpc ~ 0
PD5
Text GLabel 8825 2200 0    40   UnSpc ~ 0
PD6
Text GLabel 8825 2300 0    40   UnSpc ~ 0
PD7
Text GLabel 8825 2400 0    40   UnSpc ~ 0
PB0
Text GLabel 8825 2500 0    40   UnSpc ~ 0
PB1
Text GLabel 8825 2600 0    40   UnSpc ~ 0
PB2
Text GLabel 8825 2700 0    40   UnSpc ~ 0
PB3
Text GLabel 8825 2800 0    40   UnSpc ~ 0
PB4
$Comp
L CONN_01X08 P?
U 1 1 58EDEA17
P 9650 2450
F 0 "P?" H 9650 2900 50  0000 C CNN
F 1 "Atmega left" V 9750 2450 50  0000 C CNN
F 2 "" H 9650 2450 50  0000 C CNN
F 3 "" H 9650 2450 50  0000 C CNN
	1    9650 2450
	1    0    0    -1  
$EndComp
Text GLabel 9450 2100 0    40   UnSpc ~ 0
PD3
Text GLabel 9450 2200 0    40   UnSpc ~ 0
PD4
$Comp
L Earth #PWR?
U 1 1 58EDECB8
P 9450 2300
F 0 "#PWR?" H 9450 2050 50  0001 C CNN
F 1 "Earth" H 9450 2150 50  0001 C CNN
F 2 "" H 9450 2300 50  0000 C CNN
F 3 "" H 9450 2300 50  0000 C CNN
	1    9450 2300
	0    1    1    0   
$EndComp
Text GLabel 9450 2400 0    39   UnSpc ~ 0
3V3
$Comp
L Earth #PWR?
U 1 1 58EDEF3B
P 9450 2500
F 0 "#PWR?" H 9450 2250 50  0001 C CNN
F 1 "Earth" H 9450 2350 50  0001 C CNN
F 2 "" H 9450 2500 50  0000 C CNN
F 3 "" H 9450 2500 50  0000 C CNN
	1    9450 2500
	0    1    1    0   
$EndComp
Text GLabel 9450 2600 0    39   UnSpc ~ 0
3V3
Text GLabel 9450 2700 0    40   UnSpc ~ 0
PB6
Text GLabel 9450 2800 0    40   UnSpc ~ 0
PB7
Text GLabel 3450 1150 2    40   UnSpc ~ 0
PB0
Text GLabel 3450 1250 2    40   UnSpc ~ 0
PB1
Text GLabel 3550 1350 2    40   UnSpc ~ 0
PB2
Text GLabel 3450 1750 2    40   UnSpc ~ 0
PB6
Text GLabel 3450 1850 2    40   UnSpc ~ 0
PB7
Text GLabel 3450 2000 2    40   UnSpc ~ 0
PC0
Text GLabel 3450 2100 2    40   UnSpc ~ 0
PC1
Text GLabel 3450 2200 2    40   UnSpc ~ 0
PC2
Text GLabel 3450 2950 2    40   UnSpc ~ 0
PD2
Text GLabel 3450 3050 2    40   UnSpc ~ 0
PD3
Text GLabel 3450 3150 2    40   UnSpc ~ 0
PD4
Text GLabel 3450 3250 2    40   UnSpc ~ 0
PD5
Text GLabel 3450 3350 2    40   UnSpc ~ 0
PD6
Text GLabel 3450 3450 2    40   UnSpc ~ 0
PD7
Text GLabel 3450 2300 2    40   UnSpc ~ 0
PC3
Text GLabel 3450 2400 2    40   UnSpc ~ 0
PC4
Text GLabel 3450 2500 2    40   UnSpc ~ 0
PC5
Text GLabel 3450 2600 2    40   UnSpc ~ 0
PC6
Text GLabel 3550 1650 2    40   UnSpc ~ 0
PB5
Text GLabel 3550 1550 2    40   UnSpc ~ 0
PB4
Text GLabel 3550 1450 2    40   UnSpc ~ 0
PB3
Wire Wire Line
	1550 3250 1325 3250
Wire Wire Line
	1550 3350 1325 3350
Wire Wire Line
	1325 3450 1550 3450
Wire Wire Line
	1325 3250 1325 3350
Wire Wire Line
	1325 3350 1325 3450
Wire Wire Line
	1325 3450 1325 3625
Connection ~ 1325 3350
Connection ~ 1325 3450
Wire Wire Line
	1325 1250 1550 1250
Wire Wire Line
	1325 925  1325 1150
Wire Wire Line
	1325 1150 1325 1250
Wire Wire Line
	1325 1250 1325 1450
Wire Wire Line
	1325 1150 1550 1150
Connection ~ 1325 1150
Wire Wire Line
	8175 3875 7400 3875
Wire Wire Line
	7400 3875 7400 4025
Wire Wire Line
	8175 4575 8175 4925
Wire Wire Line
	8175 4925 8175 5600
Wire Wire Line
	8175 4475 8100 4475
Wire Wire Line
	8100 4475 8100 5000
Wire Wire Line
	8100 5000 8100 5600
Wire Wire Line
	8175 4375 8025 4375
Wire Wire Line
	8025 4375 8025 5075
Wire Wire Line
	8025 5075 8025 5600
Wire Wire Line
	8175 4275 7950 4275
Wire Wire Line
	7950 4275 7950 5150
Wire Wire Line
	7950 5150 7950 5600
Wire Wire Line
	8175 4175 7875 4175
Wire Wire Line
	7875 4175 7875 5225
Wire Wire Line
	7875 5225 7875 5600
Wire Wire Line
	8175 4075 7800 4075
Wire Wire Line
	7800 4075 7800 5300
Wire Wire Line
	7800 5300 7800 5600
Wire Wire Line
	8175 3975 7725 3975
Wire Wire Line
	7725 3975 7725 5375
Wire Wire Line
	7725 5375 7725 5600
Wire Wire Line
	9875 4575 9875 4925
Wire Wire Line
	9875 4475 9950 4475
Wire Wire Line
	9950 4475 9950 5000
Wire Wire Line
	9875 4375 10025 4375
Wire Wire Line
	10025 4375 10025 5075
Wire Wire Line
	9875 4275 10100 4275
Wire Wire Line
	10100 4275 10100 5150
Wire Wire Line
	9875 4175 10175 4175
Wire Wire Line
	10175 4175 10175 5225
Wire Wire Line
	9875 4075 10250 4075
Wire Wire Line
	10250 4075 10250 5300
Wire Wire Line
	9875 3975 10325 3975
Wire Wire Line
	10325 3975 10325 5375
Wire Wire Line
	9875 4925 8175 4925
Wire Wire Line
	9950 5000 8100 5000
Connection ~ 8100 5000
Wire Wire Line
	10025 5075 8025 5075
Connection ~ 8025 5075
Wire Wire Line
	10100 5150 7950 5150
Connection ~ 7950 5150
Wire Wire Line
	10175 5225 7875 5225
Connection ~ 7875 5225
Wire Wire Line
	10250 5300 7800 5300
Connection ~ 7800 5300
Wire Wire Line
	10325 5375 7725 5375
Connection ~ 7725 5375
Wire Wire Line
	9875 3875 9875 3650
Wire Wire Line
	9875 3650 6700 3650
Wire Wire Line
	6700 3650 6700 4025
Connection ~ 8175 4925
Wire Wire Line
	1200 1450 1325 1450
Wire Wire Line
	1325 1450 1550 1450
Connection ~ 1325 1250
Wire Wire Line
	1200 1750 1425 1750
Wire Wire Line
	1425 1750 1550 1750
Wire Wire Line
	4950 2050 5275 2050
Wire Wire Line
	5275 2050 5700 2050
Wire Wire Line
	5575 2200 5700 2200
Wire Wire Line
	5575 1325 5575 1550
Wire Wire Line
	5575 1550 5575 2200
Wire Wire Line
	4950 1550 5275 1550
Wire Wire Line
	5275 1550 5575 1550
Wire Wire Line
	5275 1550 5275 1675
Connection ~ 5575 1550
Wire Wire Line
	4950 1550 4950 1675
Connection ~ 5275 1550
Wire Wire Line
	4950 1875 4950 2050
Wire Wire Line
	4950 2050 4950 2275
Connection ~ 4950 2050
Wire Wire Line
	5275 1875 5275 2050
Connection ~ 5275 2050
Wire Wire Line
	3450 1650 3450 1700
Wire Wire Line
	3450 1700 3775 1700
Wire Wire Line
	3775 1700 3775 1650
Wire Wire Line
	3775 1650 3875 1650
Wire Wire Line
	3875 1550 3775 1550
Wire Wire Line
	3775 1550 3775 1600
Wire Wire Line
	3775 1600 3450 1600
Wire Wire Line
	3450 1600 3450 1550
Wire Wire Line
	3875 1450 3775 1450
Wire Wire Line
	3775 1450 3775 1500
Wire Wire Line
	3775 1500 3450 1500
Wire Wire Line
	3450 1500 3450 1450
Wire Wire Line
	3450 1450 3550 1450
Wire Wire Line
	3450 1550 3550 1550
Wire Wire Line
	3550 1650 3450 1650
Connection ~ 3450 1650
Connection ~ 3450 1550
Connection ~ 3450 1450
Text GLabel 3550 2750 2    40   UnSpc ~ 0
PD0
Wire Wire Line
	3450 2750 3450 2800
Wire Wire Line
	3450 2800 3775 2800
Wire Wire Line
	3775 2800 3775 2750
Wire Wire Line
	3775 2750 3900 2750
Wire Wire Line
	3450 2750 3550 2750
Connection ~ 3450 2750
Text GLabel 3550 2850 2    40   UnSpc ~ 0
PD1
Wire Wire Line
	3450 2850 3550 2850
Connection ~ 3450 2850
Wire Wire Line
	3450 2850 3450 2900
Wire Wire Line
	3450 2900 3775 2900
Wire Wire Line
	3775 2900 3775 2850
Wire Wire Line
	3775 2850 3900 2850
Text GLabel 1550 2500 0    40   UnSpc ~ 0
ADC6
Text GLabel 1550 2600 0    40   UnSpc ~ 0
ADC7
Wire Wire Line
	1425 1750 1425 1825
Connection ~ 1425 1750
Text GLabel 1200 1750 0    40   UnSpc ~ 0
AREF
Text GLabel 1200 1450 0    40   UnSpc ~ 0
AVCC
Connection ~ 1325 1450
Text GLabel 3875 1350 2    40   Output ~ 0
NRF_CSN
Wire Wire Line
	3450 1350 3550 1350
Wire Wire Line
	3450 1350 3450 1400
Wire Wire Line
	3450 1400 3775 1400
Wire Wire Line
	3775 1400 3775 1350
Wire Wire Line
	3775 1350 3875 1350
$Comp
L D_Schottky_Small D?
U 1 1 58ED73D3
P 3050 6575
F 0 "D?" H 3000 6655 50  0000 L CNN
F 1 "MBR0520LT1GOSCT-ND" H 2650 6475 50  0000 L CNN
F 2 "Diodes_SMD:SOD-123" V 3050 6575 50  0001 C CNN
F 3 "" V 3050 6575 50  0000 C CNN
	1    3050 6575
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky_Small D?
U 1 1 58ED73D4
P 3050 4875
F 0 "D?" H 3000 4775 50  0000 L CNN
F 1 "MBR0520LT1GOSCT-ND" H 2650 4975 50  0000 L CNN
F 2 "Diodes_SMD:SOD-123" V 3050 4875 50  0001 C CNN
F 3 "" V 3050 4875 50  0000 C CNN
	1    3050 4875
	1    0    0    -1  
$EndComp
$Comp
L LT1944 RG?
U 1 1 58ED73D6
P 3100 5625
F 0 "RG?" H 3050 5425 50  0000 C CNN
F 1 "LT1944EMS#PBF" H 3100 5625 50  0000 C CNN
F 2 "Housings_SSOP:MSOP-10_3x3mm_Pitch0.5mm" H 3100 5625 50  0001 C CNN
F 3 "" H 3100 5625 50  0000 C CNN
	1    3100 5625
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73D7
P 3750 5925
F 0 "#PWR?" H 3750 5675 50  0001 C CNN
F 1 "Earth" H 3750 5775 50  0001 C CNN
F 2 "" H 3750 5925 50  0000 C CNN
F 3 "" H 3750 5925 50  0000 C CNN
	1    3750 5925
	0    -1   -1   0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73D8
P 2400 5725
F 0 "#PWR?" H 2400 5475 50  0001 C CNN
F 1 "Earth" H 2400 5575 50  0001 C CNN
F 2 "" H 2400 5725 50  0000 C CNN
F 3 "" H 2400 5725 50  0000 C CNN
	1    2400 5725
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73D9
P 3750 5525
F 0 "#PWR?" H 3750 5275 50  0001 C CNN
F 1 "Earth" H 3750 5375 50  0001 C CNN
F 2 "" H 3750 5525 50  0000 C CNN
F 3 "" H 3750 5525 50  0000 C CNN
	1    3750 5525
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73DA
P 3750 5825
F 0 "C?" H 3850 5775 50  0000 L CNN
F 1 "10µF" H 3850 5875 50  0000 L TNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3750 5825 50  0001 C CNN
F 3 "" H 3750 5825 50  0000 C CNN
	1    3750 5825
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58ED73DC
P 2200 5125
F 0 "R?" H 2050 5075 50  0000 L CNN
F 1 "1M" H 2050 5175 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 2200 5125 50  0001 C CNN
F 3 "" H 2200 5125 50  0000 C CNN
	1    2200 5125
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73DD
P 2400 6325
F 0 "C?" H 2500 6275 50  0000 L CNN
F 1 "4.7pF" H 2500 6375 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2400 6325 50  0001 C CNN
F 3 "" H 2400 6325 50  0000 C CNN
	1    2400 6325
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58ED73DE
P 2200 6325
F 0 "R?" H 2050 6275 50  0000 L CNN
F 1 "1M" H 2050 6375 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 2200 6325 50  0001 C CNN
F 3 "" H 2200 6325 50  0000 C CNN
	1    2200 6325
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73DF
P 2400 5125
F 0 "C?" H 2500 5075 50  0000 L CNN
F 1 "4.7pF" H 2500 5175 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2400 5125 50  0001 C CNN
F 3 "" H 2400 5125 50  0000 C CNN
	1    2400 5125
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 58ED73E0
P 1800 5325
F 0 "R?" V 1900 5275 50  0000 L CNN
F 1 "324K" V 1700 5225 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1800 5325 50  0001 C CNN
F 3 "" H 1800 5325 50  0000 C CNN
	1    1800 5325
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73E1
P 1700 5325
F 0 "#PWR?" H 1700 5075 50  0001 C CNN
F 1 "Earth" H 1700 5175 50  0001 C CNN
F 2 "" H 1700 5325 50  0000 C CNN
F 3 "" H 1700 5325 50  0000 C CNN
	1    1700 5325
	0    1    1    0   
$EndComp
$Comp
L R_Small R?
U 1 1 58ED73E2
P 1800 6125
F 0 "R?" V 1900 6075 50  0000 L CNN
F 1 "604K" V 1700 6025 50  0000 L CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 1800 6125 50  0001 C CNN
F 3 "" H 1800 6125 50  0000 C CNN
	1    1800 6125
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73E3
P 1700 6125
F 0 "#PWR?" H 1700 5875 50  0001 C CNN
F 1 "Earth" H 1700 5975 50  0001 C CNN
F 2 "" H 1700 6125 50  0000 C CNN
F 3 "" H 1700 6125 50  0000 C CNN
	1    1700 6125
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73E4
P 1800 4875
F 0 "C?" V 1950 4825 50  0000 L CNN
F 1 "10µF" V 1650 4775 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1800 4875 50  0001 C CNN
F 3 "" H 1800 4875 50  0000 C CNN
	1    1800 4875
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73E5
P 1800 6575
F 0 "C?" V 1950 6525 50  0000 L CNN
F 1 "10µF" V 1650 6475 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1800 6575 50  0001 C CNN
F 3 "" H 1800 6575 50  0000 C CNN
	1    1800 6575
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73E6
P 1700 6575
F 0 "#PWR?" H 1700 6325 50  0001 C CNN
F 1 "Earth" H 1700 6425 50  0001 C CNN
F 2 "" H 1700 6575 50  0000 C CNN
F 3 "" H 1700 6575 50  0000 C CNN
	1    1700 6575
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73E7
P 1700 4875
F 0 "#PWR?" H 1700 4625 50  0001 C CNN
F 1 "Earth" H 1700 4725 50  0001 C CNN
F 2 "" H 1700 4875 50  0000 C CNN
F 3 "" H 1700 4875 50  0000 C CNN
	1    1700 4875
	0    1    1    0   
$EndComp
Text GLabel 2000 4475 1    60   Output ~ 0
5V0
Text GLabel 1850 7225 0    60   Output ~ 0
3V3
$Comp
L VCC #PWR?
U 1 1 58ED73E8
P 4250 6125
F 0 "#PWR?" H 4250 5975 50  0001 C CNN
F 1 "VCC" V 4250 6325 50  0000 C CNN
F 2 "" H 4250 6125 50  0000 C CNN
F 3 "" H 4250 6125 50  0000 C CNN
	1    4250 6125
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 58ED73E9
P 4000 5725
F 0 "#PWR?" H 4000 5575 50  0001 C CNN
F 1 "VCC" V 4000 5925 50  0000 C CNN
F 2 "" H 4000 5725 50  0000 C CNN
F 3 "" H 4000 5725 50  0000 C CNN
	1    4000 5725
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 58ED73EA
P 4250 5325
F 0 "#PWR?" H 4250 5175 50  0001 C CNN
F 1 "VCC" V 4250 5525 50  0000 C CNN
F 2 "" H 4250 5325 50  0000 C CNN
F 3 "" H 4250 5325 50  0000 C CNN
	1    4250 5325
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 58ED73EB
P 2400 5525
F 0 "#PWR?" H 2400 5375 50  0001 C CNN
F 1 "VCC" V 2400 5725 50  0000 C CNN
F 2 "" H 2400 5525 50  0000 C CNN
F 3 "" H 2400 5525 50  0000 C CNN
	1    2400 5525
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 58ED73EC
P 2400 5925
F 0 "#PWR?" H 2400 5775 50  0001 C CNN
F 1 "VCC" V 2400 6125 50  0000 C CNN
F 2 "" H 2400 5925 50  0000 C CNN
F 3 "" H 2400 5925 50  0000 C CNN
	1    2400 5925
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG?
U 1 1 58ED73ED
P 1850 6925
F 0 "#FLG?" H 1850 7020 50  0001 C CNN
F 1 "PWR_FLAG" V 1750 7025 50  0000 C CNN
F 2 "" H 1850 6925 50  0000 C CNN
F 3 "" H 1850 6925 50  0000 C CNN
	1    1850 6925
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG?
U 1 1 58ED73EE
P 2400 4475
F 0 "#FLG?" H 2400 4570 50  0001 C CNN
F 1 "PWR_FLAG" H 2400 4655 50  0000 C CNN
F 2 "" H 2400 4475 50  0000 C CNN
F 3 "" H 2400 4475 50  0000 C CNN
	1    2400 4475
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73EF
P 2700 6925
F 0 "C?" V 2650 7025 50  0000 L CNN
F 1 "100uF" V 2650 6625 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2700 6925 50  0001 C CNN
F 3 "" H 2700 6925 50  0000 C CNN
	1    2700 6925
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73F0
P 2700 7325
F 0 "C?" V 2650 7425 50  0000 L CNN
F 1 "100uF" V 2650 7025 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2700 7325 50  0001 C CNN
F 3 "" H 2700 7325 50  0000 C CNN
	1    2700 7325
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73F1
P 3050 6925
F 0 "#PWR?" H 3050 6675 50  0001 C CNN
F 1 "Earth" H 3050 6775 50  0001 C CNN
F 2 "" H 3050 6925 50  0000 C CNN
F 3 "" H 3050 6925 50  0000 C CNN
	1    3050 6925
	0    -1   -1   0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73F2
P 3050 7325
F 0 "#PWR?" H 3050 7075 50  0001 C CNN
F 1 "Earth" H 3050 7175 50  0001 C CNN
F 2 "" H 3050 7325 50  0000 C CNN
F 3 "" H 3050 7325 50  0000 C CNN
	1    3050 7325
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C?
U 1 1 58ED73F3
P 2700 7125
F 0 "C?" V 2650 7225 50  0000 L CNN
F 1 "100uF" V 2650 6825 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2700 7125 50  0001 C CNN
F 3 "" H 2700 7125 50  0000 C CNN
	1    2700 7125
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR?
U 1 1 58ED73F4
P 3050 7125
F 0 "#PWR?" H 3050 6875 50  0001 C CNN
F 1 "Earth" H 3050 6975 50  0001 C CNN
F 2 "" H 3050 7125 50  0000 C CNN
F 3 "" H 3050 7125 50  0000 C CNN
	1    3050 7125
	0    -1   -1   0   
$EndComp
Connection ~ 3750 5925
Connection ~ 3750 5725
Connection ~ 2400 5325
Connection ~ 3750 6125
Connection ~ 3750 5325
Wire Wire Line
	3750 4875 3750 5325
Wire Wire Line
	3150 4875 3750 4875
Wire Wire Line
	3750 6575 3750 6125
Wire Wire Line
	3150 6575 3750 6575
Wire Wire Line
	1900 6575 2200 6575
Wire Wire Line
	2200 6575 2400 6575
Wire Wire Line
	2400 6575 2950 6575
Wire Wire Line
	1900 6125 2200 6125
Wire Wire Line
	2200 6125 2400 6125
Wire Wire Line
	1900 5325 2200 5325
Wire Wire Line
	2200 5325 2400 5325
Wire Wire Line
	3750 5725 4000 5725
Wire Wire Line
	2200 5225 2200 5325
Wire Wire Line
	2400 5325 2400 5225
Wire Wire Line
	2200 4675 2200 4875
Wire Wire Line
	2200 4875 2200 5025
Wire Wire Line
	2400 4875 2400 5025
Wire Wire Line
	2400 6575 2400 6425
Wire Wire Line
	2400 6125 2400 6225
Wire Wire Line
	2200 6225 2200 6125
Wire Wire Line
	2200 6425 2200 6575
Wire Wire Line
	2200 6575 2200 6925
Wire Wire Line
	2200 6925 2200 7225
Wire Wire Line
	1900 4875 2200 4875
Wire Wire Line
	2200 4875 2400 4875
Wire Wire Line
	2400 4875 2950 4875
Connection ~ 2200 6125
Connection ~ 2200 6575
Connection ~ 2400 6125
Connection ~ 2200 5325
Connection ~ 2200 4875
Connection ~ 2400 4875
Connection ~ 2400 6575
Wire Wire Line
	2000 4675 2200 4675
Wire Wire Line
	2200 4675 2400 4675
Wire Wire Line
	2400 4675 2400 4475
Wire Wire Line
	2000 4675 2000 4475
Connection ~ 2200 4675
Wire Wire Line
	2200 7225 1850 7225
Wire Wire Line
	1850 6925 2200 6925
Wire Wire Line
	2200 6925 2350 6925
Wire Wire Line
	2350 6925 2600 6925
Connection ~ 2200 6925
Wire Wire Line
	2350 6925 2350 7125
Wire Wire Line
	2350 7125 2350 7325
Wire Wire Line
	2350 7325 2600 7325
Connection ~ 2350 6925
Wire Wire Line
	2800 6925 3050 6925
Wire Wire Line
	2800 7325 3050 7325
Wire Wire Line
	2350 7125 2600 7125
Connection ~ 2350 7125
Wire Wire Line
	2800 7125 3050 7125
$Comp
L L_Small L?
U 1 1 58ED98F1
P 4000 5325
F 0 "L?" V 4100 5300 50  0000 L CNN
F 1 "4.7µH LQH5BPB4R7NT0L" V 4200 4625 50  0000 L CNN
F 2 "" H 4000 5325 50  0000 C CNN
F 3 "" H 4000 5325 50  0000 C CNN
	1    4000 5325
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3750 5325 3900 5325
Wire Wire Line
	4100 5325 4250 5325
$Comp
L L_Small L?
U 1 1 58EDA41D
P 4025 6125
F 0 "L?" V 3925 6100 50  0000 L CNN
F 1 "4.7µH LQH5BPB4R7NT0L" V 3800 5450 50  0000 L CNN
F 2 "" H 4025 6125 50  0000 C CNN
F 3 "" H 4025 6125 50  0000 C CNN
	1    4025 6125
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4250 6125 4125 6125
Wire Wire Line
	3750 6125 3925 6125
$EndSCHEMATC
