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
LIBS:switches
LIBS:74xgxx
LIBS:cd4072bm96
LIBS:ldd-c514ri
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
Text GLabel 1375 2125 1    39   Input ~ 0
3V3
$Comp
L Earth #PWR01
U 1 1 58EBB46B
P 1375 5150
F 0 "#PWR01" H 1375 4900 50  0001 C CNN
F 1 "Earth" H 1375 5000 50  0001 C CNN
F 2 "" H 1375 5150 50  0000 C CNN
F 3 "" H 1375 5150 50  0000 C CNN
	1    1375 5150
	1    0    0    -1  
$EndComp
$Comp
L BSS138 Q1
U 1 1 58EBBE8D
P 6825 4675
F 0 "Q1" V 6975 4075 50  0000 L CNN
F 1 "BSS214NWH6327XTSA1" V 7050 4075 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70_Handsoldering" H 7025 4600 50  0001 L CIN
F 3 "" H 6825 4675 50  0000 L CNN
	1    6825 4675
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR02
U 1 1 58EBC7E8
P 6925 5000
F 0 "#PWR02" H 6925 4750 50  0001 C CNN
F 1 "Earth" H 6925 4850 50  0001 C CNN
F 2 "" H 6925 5000 50  0000 C CNN
F 3 "" H 6925 5000 50  0000 C CNN
	1    6925 5000
	1    0    0    -1  
$EndComp
Text GLabel 6625 4300 1    40   Input ~ 0
D9
Text GLabel 8250 6250 3    40   Input ~ 0
A1
$Comp
L R_Small R12
U 1 1 58EBCC7E
P 8250 6150
F 0 "R12" V 8250 6125 24  0000 L CNN
F 1 "130" V 8225 6225 24  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8250 6150 50  0001 C CNN
F 3 "" H 8250 6150 50  0000 C CNN
	1    8250 6150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R13
U 1 1 58EBCDE7
P 8325 6150
F 0 "R13" V 8325 6125 24  0000 L CNN
F 1 "130" V 8300 6225 24  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8325 6150 50  0001 C CNN
F 3 "" H 8325 6150 50  0000 C CNN
	1    8325 6150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R15
U 1 1 58EBCE0F
P 8400 6150
F 0 "R15" V 8400 6125 24  0000 L CNN
F 1 "130" V 8375 6225 24  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8400 6150 50  0001 C CNN
F 3 "" H 8400 6150 50  0000 C CNN
	1    8400 6150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R17
U 1 1 58EBCE36
P 8475 6150
F 0 "R17" V 8475 6125 24  0000 L CNN
F 1 "130" V 8450 6225 24  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8475 6150 50  0001 C CNN
F 3 "" H 8475 6150 50  0000 C CNN
	1    8475 6150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R18
U 1 1 58EBCE66
P 8550 6150
F 0 "R18" V 8550 6125 24  0000 L CNN
F 1 "130" V 8525 6225 24  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8550 6150 50  0001 C CNN
F 3 "" H 8550 6150 50  0000 C CNN
	1    8550 6150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R20
U 1 1 58EBCE93
P 8625 6150
F 0 "R20" V 8625 6125 24  0000 L CNN
F 1 "130" V 8600 6225 24  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8625 6150 50  0001 C CNN
F 3 "" H 8625 6150 50  0000 C CNN
	1    8625 6150
	1    0    0    -1  
$EndComp
$Comp
L R_Small R22
U 1 1 58EBCFC3
P 8700 6150
F 0 "R22" V 8700 6125 24  0000 L CNN
F 1 "130" V 8675 6225 24  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8700 6150 50  0001 C CNN
F 3 "" H 8700 6150 50  0000 C CNN
	1    8700 6150
	1    0    0    -1  
$EndComp
Text GLabel 8325 6250 3    40   Input ~ 0
A2
Text GLabel 8400 6250 3    40   Input ~ 0
A3
Text GLabel 8475 6250 3    40   Input ~ 0
SDA
Text GLabel 8550 6250 3    40   Input ~ 0
SCL
Text GLabel 8625 6250 3    40   Input ~ 0
D6
Text GLabel 8700 6250 3    40   Input ~ 0
D7
Text GLabel 3500 3175 2    40   Output ~ 0
SCK
Text GLabel 3500 2975 2    40   Output ~ 0
MOSI
Text GLabel 3500 3075 2    40   Input ~ 0
MISO
Text GLabel 3500 4275 2    40   Input ~ 0
RXI
Text GLabel 3500 4375 2    40   Output ~ 0
TXO
$Comp
L C_Small C7
U 1 1 58EC20DD
P 1475 3450
F 0 "C7" V 1600 3400 50  0000 L CNN
F 1 "100nF" V 1350 3350 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1475 3450 50  0001 C CNN
F 3 "" H 1475 3450 50  0000 C CNN
	1    1475 3450
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR03
U 1 1 58EC234C
P 1475 3550
F 0 "#PWR03" H 1475 3300 50  0001 C CNN
F 1 "Earth" H 1475 3400 50  0001 C CNN
F 2 "" H 1475 3550 50  0000 C CNN
F 3 "" H 1475 3550 50  0000 C CNN
	1    1475 3550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P3
U 1 1 58EC2CAD
P 2450 1150
F 0 "P3" H 2450 1500 50  0000 C CNN
F 1 "Programming header" V 2550 1150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 2450 1150 50  0001 C CNN
F 3 "" H 2450 1150 50  0000 C CNN
	1    2450 1150
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR04
U 1 1 58EC2D79
P 2250 900
F 0 "#PWR04" H 2250 650 50  0001 C CNN
F 1 "Earth" H 2250 750 50  0001 C CNN
F 2 "" H 2250 900 50  0000 C CNN
F 3 "" H 2250 900 50  0000 C CNN
	1    2250 900 
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR05
U 1 1 58EC2E45
P 2250 1000
F 0 "#PWR05" H 2250 750 50  0001 C CNN
F 1 "Earth" H 2250 850 50  0001 C CNN
F 2 "" H 2250 1000 50  0000 C CNN
F 3 "" H 2250 1000 50  0000 C CNN
	1    2250 1000
	0    1    1    0   
$EndComp
Text GLabel 2250 1100 0    40   Output ~ 0
3V3
Text GLabel 2250 1200 0    40   Output ~ 0
RXI
Text GLabel 2250 1300 0    40   Input ~ 0
TXO
Text GLabel 2250 1400 0    40   Output ~ 0
DTR
$Comp
L nRF24L01+ U1
U 1 1 58ED73BF
P 5775 6675
F 0 "U1" H 5775 6375 50  0001 C CNN
F 1 "nRF24L01+" H 5775 6975 50  0000 C CNN
F 2 "mysensors_radios:NRF24L01" H 5775 6775 50  0001 C CNN
F 3 "DOCUMENTATION" H 5775 6625 50  0001 C CNN
	1    5775 6675
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR06
U 1 1 58ED7961
P 4600 6575
F 0 "#PWR06" H 4600 6325 50  0001 C CNN
F 1 "Earth" H 4600 6425 50  0001 C CNN
F 2 "" H 4600 6575 50  0000 C CNN
F 3 "" H 4600 6575 50  0000 C CNN
	1    4600 6575
	1    0    0    -1  
$EndComp
Text GLabel 4600 5875 1    40   Input ~ 0
3V3
$Comp
L C_Small C14
U 1 1 58ED7C7A
P 4600 6200
F 0 "C14" H 4610 6270 50  0000 L CNN
F 1 "10uF" H 4610 6120 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4600 6200 50  0001 C CNN
F 3 "" H 4600 6200 50  0000 C CNN
	1    4600 6200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C12
U 1 1 58ED7CF5
P 4275 6200
F 0 "C12" H 4285 6270 50  0000 L CNN
F 1 "100nF" H 4285 6120 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4275 6200 50  0001 C CNN
F 3 "" H 4275 6200 50  0000 C CNN
	1    4275 6200
	1    0    0    -1  
$EndComp
Text GLabel 6525 6475 2    40   Input ~ 0
SCK
Text GLabel 6525 6625 2    40   Input ~ 0
MOSI
Text GLabel 6525 6775 2    40   Output ~ 0
MISO
NoConn ~ 6525 6925
Text GLabel 4500 6775 0    40   Input ~ 0
D2
Text GLabel 4500 6925 0    40   Input ~ 0
D3
$Comp
L CONN_01X08 P2
U 1 1 58EDCA87
P 1775 1200
F 0 "P2" H 1775 1650 50  0000 C CNN
F 1 "Atmega top" V 1875 1200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 1775 1200 50  0001 C CNN
F 3 "" H 1775 1200 50  0000 C CNN
	1    1775 1200
	1    0    0    -1  
$EndComp
Text GLabel 1575 850  0    40   UnSpc ~ 0
D2
Text GLabel 1575 950  0    40   UnSpc ~ 0
TXO
Text GLabel 1575 1050 0    40   UnSpc ~ 0
RXI
Text GLabel 1575 1150 0    40   UnSpc ~ 0
RESET
Text GLabel 1575 1250 0    40   UnSpc ~ 0
SCL
Text GLabel 1575 1350 0    40   UnSpc ~ 0
SDA
Text GLabel 1575 1450 0    40   UnSpc ~ 0
A3
Text GLabel 1575 1550 0    40   UnSpc ~ 0
A2
$Comp
L CONN_01X08 P1
U 1 1 58EDD361
P 1050 1175
F 0 "P1" H 1050 1625 50  0000 C CNN
F 1 "Atmega right" V 1150 1175 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 1050 1175 50  0001 C CNN
F 3 "" H 1050 1175 50  0000 C CNN
	1    1050 1175
	1    0    0    -1  
$EndComp
Text GLabel 850  825  0    40   UnSpc ~ 0
A1
Text GLabel 850  925  0    40   UnSpc ~ 0
A0
Text GLabel 850  1025 0    40   UnSpc ~ 0
A7
$Comp
L Earth #PWR07
U 1 1 58EDD62C
P 850 1125
F 0 "#PWR07" H 850 875 50  0001 C CNN
F 1 "Earth" H 850 975 50  0001 C CNN
F 2 "" H 850 1125 50  0000 C CNN
F 3 "" H 850 1125 50  0000 C CNN
	1    850  1125
	0    1    1    0   
$EndComp
Text GLabel 850  1225 0    40   UnSpc ~ 0
AREF
Text GLabel 850  1325 0    40   UnSpc ~ 0
A6
Text GLabel 850  1425 0    40   UnSpc ~ 0
AVCC
Text GLabel 850  1525 0    40   UnSpc ~ 0
SCK
$Comp
L CONN_01X08 P4
U 1 1 58EDD9D5
P 3175 1175
F 0 "P4" H 3175 1625 50  0000 C CNN
F 1 "Atmega bottom" V 3275 1175 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 3175 1175 50  0001 C CNN
F 3 "" H 3175 1175 50  0000 C CNN
	1    3175 1175
	1    0    0    -1  
$EndComp
Text GLabel 2975 825  0    40   UnSpc ~ 0
D5
Text GLabel 2975 925  0    40   UnSpc ~ 0
D6
Text GLabel 2975 1025 0    40   UnSpc ~ 0
D7
Text GLabel 2975 1125 0    40   UnSpc ~ 0
D8
Text GLabel 2975 1225 0    40   UnSpc ~ 0
D9
Text GLabel 2975 1325 0    40   UnSpc ~ 0
D10
Text GLabel 2975 1425 0    40   UnSpc ~ 0
MOSI
Text GLabel 2975 1525 0    40   UnSpc ~ 0
MISO
$Comp
L CONN_01X04 P5
U 1 1 58EDEA17
P 3750 1175
F 0 "P5" H 3750 1525 50  0000 C CNN
F 1 "Atmega left" V 3850 1175 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 3750 1175 50  0001 C CNN
F 3 "" H 3750 1175 50  0000 C CNN
	1    3750 1175
	1    0    0    -1  
$EndComp
Text GLabel 3550 1025 0    40   UnSpc ~ 0
D3
Text GLabel 3550 1125 0    40   UnSpc ~ 0
D4
$Comp
L Earth #PWR08
U 1 1 58EDECB8
P 3550 1225
F 0 "#PWR08" H 3550 975 50  0001 C CNN
F 1 "Earth" H 3550 1075 50  0001 C CNN
F 2 "" H 3550 1225 50  0000 C CNN
F 3 "" H 3550 1225 50  0000 C CNN
	1    3550 1225
	0    1    1    0   
$EndComp
Text GLabel 3550 1325 0    39   UnSpc ~ 0
3V3
Text GLabel 3500 2675 2    40   BiDi ~ 0
D8
Text GLabel 3500 2775 2    40   BiDi ~ 0
D9
Text GLabel 3500 3275 2    40   UnSpc ~ 0
XTAL1
Text GLabel 3500 3375 2    40   UnSpc ~ 0
XTAL2
Text GLabel 3500 3525 2    40   Input ~ 0
A0
Text GLabel 3500 3625 2    40   Input ~ 0
A1
Text GLabel 3500 3725 2    40   Input ~ 0
A2
Text GLabel 3500 4475 2    40   BiDi ~ 0
D2
Text GLabel 3500 4575 2    40   BiDi ~ 0
D3
Text GLabel 3500 4675 2    40   BiDi ~ 0
D4
Text GLabel 3500 4775 2    40   BiDi ~ 0
D5
Text GLabel 3500 4875 2    40   BiDi ~ 0
D6
Text GLabel 3500 4975 2    40   BiDi ~ 0
D7
Text GLabel 3500 3825 2    40   Input ~ 0
A3
Text GLabel 3500 3925 2    40   BiDi ~ 0
SDA
Text GLabel 3500 4025 2    40   Output ~ 0
SCL
Text GLabel 1600 4025 0    40   Input ~ 0
A6
Text GLabel 1600 4125 0    40   Input ~ 0
A7
Text GLabel 1250 3275 0    40   Input ~ 0
AREF
Text GLabel 1250 2975 0    40   Input ~ 0
AVCC
Text GLabel 3500 2875 2    40   BiDi ~ 0
D10
$Comp
L D_Schottky_Small D1
U 1 1 58ED73D3
P 2000 6775
F 0 "D1" H 1950 6855 50  0000 L CNN
F 1 "MBR0520LT1GOSCT-ND" H 1600 6675 50  0000 L CNN
F 2 "Diodes_SMD:D_SOD-123" V 2000 6775 50  0001 C CNN
F 3 "" V 2000 6775 50  0000 C CNN
	1    2000 6775
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR09
U 1 1 58ED73D7
P 2700 6125
F 0 "#PWR09" H 2700 5875 50  0001 C CNN
F 1 "Earth" H 2700 5975 50  0001 C CNN
F 2 "" H 2700 6125 50  0000 C CNN
F 3 "" H 2700 6125 50  0000 C CNN
	1    2700 6125
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR010
U 1 1 58ED73D8
P 1350 5925
F 0 "#PWR010" H 1350 5675 50  0001 C CNN
F 1 "Earth" H 1350 5775 50  0001 C CNN
F 2 "" H 1350 5925 50  0000 C CNN
F 3 "" H 1350 5925 50  0000 C CNN
	1    1350 5925
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR011
U 1 1 58ED73D9
P 2700 5725
F 0 "#PWR011" H 2700 5475 50  0001 C CNN
F 1 "Earth" H 2700 5575 50  0001 C CNN
F 2 "" H 2700 5725 50  0000 C CNN
F 3 "" H 2700 5725 50  0000 C CNN
	1    2700 5725
	1    0    0    -1  
$EndComp
$Comp
L C_Small C11
U 1 1 58ED73DA
P 2700 6025
F 0 "C11" H 2800 5975 50  0000 L CNN
F 1 "10µF" H 2800 6075 50  0000 L TNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2700 6025 50  0001 C CNN
F 3 "" H 2700 6025 50  0000 C CNN
	1    2700 6025
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 58ED73DD
P 1350 6525
F 0 "C6" H 1450 6475 50  0000 L CNN
F 1 "4.7pF" H 1450 6575 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1350 6525 50  0001 C CNN
F 3 "" H 1350 6525 50  0000 C CNN
	1    1350 6525
	1    0    0    -1  
$EndComp
$Comp
L R_Small R2
U 1 1 58ED73DE
P 1150 6525
F 0 "R2" H 1000 6475 50  0000 L CNN
F 1 "1M" H 1000 6575 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 1150 6525 50  0001 C CNN
F 3 "" H 1150 6525 50  0000 C CNN
	1    1150 6525
	1    0    0    -1  
$EndComp
$Comp
L R_Small R1
U 1 1 58ED73E2
P 750 6325
F 0 "R1" V 850 6275 50  0000 L CNN
F 1 "604K" V 650 6225 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 750 6325 50  0001 C CNN
F 3 "" H 750 6325 50  0000 C CNN
	1    750  6325
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR012
U 1 1 58ED73E3
P 650 6325
F 0 "#PWR012" H 650 6075 50  0001 C CNN
F 1 "Earth" H 650 6175 50  0001 C CNN
F 2 "" H 650 6325 50  0000 C CNN
F 3 "" H 650 6325 50  0000 C CNN
	1    650  6325
	1    0    0    -1  
$EndComp
$Comp
L C_Small C3
U 1 1 58ED73E5
P 750 6775
F 0 "C3" V 900 6725 50  0000 L CNN
F 1 "10µF" V 600 6675 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 750 6775 50  0001 C CNN
F 3 "" H 750 6775 50  0000 C CNN
	1    750  6775
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR013
U 1 1 58ED73E6
P 650 6775
F 0 "#PWR013" H 650 6525 50  0001 C CNN
F 1 "Earth" H 650 6625 50  0001 C CNN
F 2 "" H 650 6775 50  0000 C CNN
F 3 "" H 650 6775 50  0000 C CNN
	1    650  6775
	1    0    0    -1  
$EndComp
Text GLabel 800  7425 0    60   Output ~ 0
3V3
$Comp
L VCC #PWR014
U 1 1 58ED73EC
P 1350 6125
F 0 "#PWR014" H 1350 5975 50  0001 C CNN
F 1 "VCC" V 1350 6325 50  0000 C CNN
F 2 "" H 1350 6125 50  0000 C CNN
F 3 "" H 1350 6125 50  0000 C CNN
	1    1350 6125
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG015
U 1 1 58ED73ED
P 800 7125
F 0 "#FLG015" H 800 7220 50  0001 C CNN
F 1 "PWR_FLAG" V 700 7225 50  0000 C CNN
F 2 "" H 800 7125 50  0000 C CNN
F 3 "" H 800 7125 50  0000 C CNN
	1    800  7125
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C8
U 1 1 58ED73EF
P 1650 7125
F 0 "C8" V 1600 7225 50  0000 L CNN
F 1 "100uF" V 1600 6825 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1650 7125 50  0001 C CNN
F 3 "" H 1650 7125 50  0000 C CNN
	1    1650 7125
	0    1    1    0   
$EndComp
$Comp
L C_Small C10
U 1 1 58ED73F0
P 1650 7525
F 0 "C10" V 1600 7625 50  0000 L CNN
F 1 "100uF" V 1600 7225 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1650 7525 50  0001 C CNN
F 3 "" H 1650 7525 50  0000 C CNN
	1    1650 7525
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR016
U 1 1 58ED73F1
P 2000 7125
F 0 "#PWR016" H 2000 6875 50  0001 C CNN
F 1 "Earth" H 2000 6975 50  0001 C CNN
F 2 "" H 2000 7125 50  0000 C CNN
F 3 "" H 2000 7125 50  0000 C CNN
	1    2000 7125
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR017
U 1 1 58ED73F2
P 2000 7525
F 0 "#PWR017" H 2000 7275 50  0001 C CNN
F 1 "Earth" H 2000 7375 50  0001 C CNN
F 2 "" H 2000 7525 50  0000 C CNN
F 3 "" H 2000 7525 50  0000 C CNN
	1    2000 7525
	1    0    0    -1  
$EndComp
$Comp
L C_Small C9
U 1 1 58ED73F3
P 1650 7325
F 0 "C9" V 1600 7425 50  0000 L CNN
F 1 "100uF" V 1600 7025 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1650 7325 50  0001 C CNN
F 3 "" H 1650 7325 50  0000 C CNN
	1    1650 7325
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR018
U 1 1 58ED73F4
P 2000 7325
F 0 "#PWR018" H 2000 7075 50  0001 C CNN
F 1 "Earth" H 2000 7175 50  0001 C CNN
F 2 "" H 2000 7325 50  0000 C CNN
F 3 "" H 2000 7325 50  0000 C CNN
	1    2000 7325
	1    0    0    -1  
$EndComp
$Comp
L L_Small L1
U 1 1 58EDA41D
P 2975 6325
F 0 "L1" V 2875 6300 50  0000 L CNN
F 1 "4.7µH LQH5BPB4R7NT0L" V 2750 5650 50  0000 L CNN
F 2 "Inductors:Inductor_Taiyo-Yuden_NR-50xx_HandSoldering" H 2975 6325 50  0001 C CNN
F 3 "" H 2975 6325 50  0000 C CNN
	1    2975 6325
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R10
U 1 1 58EDD431
P 6625 4550
F 0 "R10" H 6425 4600 50  0000 L CNN
F 1 "100" H 6425 4525 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 6625 4550 50  0001 C CNN
F 3 "" H 6625 4550 50  0000 C CNN
	1    6625 4550
	1    0    0    -1  
$EndComp
$Comp
L LT1944 RG1
U 1 1 58ED73D6
P 2050 5825
F 0 "RG1" H 2000 5625 50  0000 C CNN
F 1 "LT1944EMS#PBF" H 2050 5825 50  0000 C CNN
F 2 "Housings_SSOP:MSOP-10_3x3mm_Pitch0.5mm" H 2050 5825 50  0001 C CNN
F 3 "" H 2050 5825 50  0000 C CNN
	1    2050 5825
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR019
U 1 1 58EDFF3B
P 1350 5725
F 0 "#PWR019" H 1350 5475 50  0001 C CNN
F 1 "Earth" H 1350 5575 50  0001 C CNN
F 2 "" H 1350 5725 50  0000 C CNN
F 3 "" H 1350 5725 50  0000 C CNN
	1    1350 5725
	1    0    0    -1  
$EndComp
NoConn ~ 1350 5525
NoConn ~ 2700 5525
$Comp
L Earth #PWR020
U 1 1 58EE190E
P 700 4675
F 0 "#PWR020" H 700 4425 50  0001 C CNN
F 1 "Earth" H 700 4525 50  0001 C CNN
F 2 "" H 700 4675 50  0000 C CNN
F 3 "" H 700 4675 50  0000 C CNN
	1    700  4675
	1    0    0    -1  
$EndComp
Text GLabel 700  3925 1    40   UnSpc ~ 0
XTAL1
Text GLabel 1175 3925 1    40   UnSpc ~ 0
XTAL2
$Comp
L POT RV1
U 1 1 58EE4358
P 9600 1800
F 0 "RV1" V 9425 1800 50  0000 C CNN
F 1 "POT" V 9500 1800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 9600 1800 50  0001 C CNN
F 3 "" H 9600 1800 50  0000 C CNN
	1    9600 1800
	-1   0    0    1   
$EndComp
$Comp
L Battery_Cell BT1
U 1 1 58EE51CB
P 9900 2750
F 0 "BT1" H 10000 2850 50  0000 L CNN
F 1 "Battery_Cell" H 10000 2750 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" V 9900 2810 50  0001 C CNN
F 3 "" V 9900 2810 50  0000 C CNN
	1    9900 2750
	1    0    0    -1  
$EndComp
$Comp
L ATMEGA328P-A IC1
U 1 1 58EBB3D2
P 2500 3775
F 0 "IC1" H 1750 5025 50  0000 L BNN
F 1 "ATMEGA328P-A" H 2900 2375 50  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 2500 3775 50  0001 C CIN
F 3 "" H 2500 3775 50  0000 C CNN
	1    2500 3775
	1    0    0    -1  
$EndComp
$Comp
L C_Small C13
U 1 1 58EEE558
P 4375 4125
F 0 "C13" V 4500 4075 50  0000 L CNN
F 1 "100nF" V 4250 3975 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 4375 4125 50  0001 C CNN
F 3 "" H 4375 4125 50  0000 C CNN
	1    4375 4125
	0    -1   -1   0   
$EndComp
$Comp
L R_Small R3
U 1 1 58EEEA40
P 4200 3950
F 0 "R3" H 4050 4000 50  0000 L CNN
F 1 "10K" H 4000 3900 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 4200 3950 50  0001 C CNN
F 3 "" H 4200 3950 50  0000 C CNN
	1    4200 3950
	1    0    0    -1  
$EndComp
Text GLabel 4200 3800 1    39   Input ~ 0
3V3
Text GLabel 4525 4125 2    40   Input ~ 0
DTR
$Comp
L SW_SPST SW10
U 1 1 58EF371E
P 7800 1700
F 0 "SW10" H 7725 1625 50  0000 C CNN
F 1 "SW_SPST" H 8125 1625 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7800 1700 50  0001 C CNN
F 3 "" H 7800 1700 50  0000 C CNN
	1    7800 1700
	0    1    1    0   
$EndComp
$Comp
L SW_SPST SW1
U 1 1 58EF395F
P 4200 4475
F 0 "SW1" V 4300 4650 50  0000 C CNN
F 1 "SW_RST" V 4200 4700 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_B3S-1000" H 4200 4475 50  0001 C CNN
F 3 "" H 4200 4475 50  0000 C CNN
	1    4200 4475
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR021
U 1 1 58EF3F4E
P 4200 4675
F 0 "#PWR021" H 4200 4425 50  0001 C CNN
F 1 "Earth" H 4200 4525 50  0001 C CNN
F 2 "" H 4200 4675 50  0000 C CNN
F 3 "" H 4200 4675 50  0000 C CNN
	1    4200 4675
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 58EF5F73
P 975 2550
F 0 "C4" H 985 2620 50  0000 L CNN
F 1 "100nF" H 985 2470 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 975 2550 50  0001 C CNN
F 3 "" H 975 2550 50  0000 C CNN
	1    975  2550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 58EF61B7
P 675 2550
F 0 "C1" H 685 2620 50  0000 L CNN
F 1 "10uF" H 685 2470 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 675 2550 50  0001 C CNN
F 3 "" H 675 2550 50  0000 C CNN
	1    675  2550
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR022
U 1 1 58EF64C0
P 675 2850
F 0 "#PWR022" H 675 2600 50  0001 C CNN
F 1 "Earth" H 675 2700 50  0001 C CNN
F 2 "" H 675 2850 50  0000 C CNN
F 3 "" H 675 2850 50  0000 C CNN
	1    675  2850
	1    0    0    -1  
$EndComp
$Comp
L SW_SPST SW8
U 1 1 58F00DAB
P 7475 1700
F 0 "SW8" H 7375 1625 50  0000 C CNN
F 1 "SW_SPST" H 7800 1625 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7475 1700 50  0001 C CNN
F 3 "" H 7475 1700 50  0000 C CNN
	1    7475 1700
	0    1    1    0   
$EndComp
$Comp
L Earth #PWR023
U 1 1 58F02228
P 7875 3500
F 0 "#PWR023" H 7875 3250 50  0001 C CNN
F 1 "Earth" H 7875 3350 50  0001 C CNN
F 2 "" H 7875 3500 50  0000 C CNN
F 3 "" H 7875 3500 50  0000 C CNN
	1    7875 3500
	1    0    0    -1  
$EndComp
Text GLabel 8050 2025 1    40   Output ~ 0
D4
Text GLabel 8150 2025 1    40   Output ~ 0
D5
$Comp
L R_Small R14
U 1 1 58F06A3E
P 8050 3100
F 0 "R14" V 8100 2875 50  0000 L CNN
F 1 "1M" V 8100 3200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8050 3100 50  0001 C CNN
F 3 "" H 8050 3100 50  0000 C CNN
	1    8050 3100
	-1   0    0    1   
$EndComp
$Comp
L R_Small R16
U 1 1 58F0776A
P 8150 3100
F 0 "R16" V 8200 2875 50  0000 L CNN
F 1 "1M" V 8200 3200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8150 3100 50  0001 C CNN
F 3 "" H 8150 3100 50  0000 C CNN
	1    8150 3100
	-1   0    0    1   
$EndComp
$Comp
L R_Small R19
U 1 1 58F07820
P 8250 3100
F 0 "R19" V 8300 2875 50  0000 L CNN
F 1 "1M" V 8300 3200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8250 3100 50  0001 C CNN
F 3 "" H 8250 3100 50  0000 C CNN
	1    8250 3100
	-1   0    0    1   
$EndComp
Text GLabel 8325 1150 1    39   Input ~ 0
3V3
Text GLabel 3900 4250 3    40   Input ~ 0
RESET
$Comp
L Earth #PWR024
U 1 1 58EBC7CA
P 7975 5000
F 0 "#PWR024" H 7975 4750 50  0001 C CNN
F 1 "Earth" H 7975 4850 50  0001 C CNN
F 2 "" H 7975 5000 50  0000 C CNN
F 3 "" H 7975 5000 50  0000 C CNN
	1    7975 5000
	1    0    0    -1  
$EndComp
$Comp
L R_Small R11
U 1 1 58EDD9A8
P 7675 4550
F 0 "R11" H 7475 4600 50  0000 L CNN
F 1 "100" H 7475 4525 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 7675 4550 50  0001 C CNN
F 3 "" H 7675 4550 50  0000 C CNN
	1    7675 4550
	1    0    0    -1  
$EndComp
$Comp
L BSS138 Q2
U 1 1 58EBBDC9
P 7875 4675
F 0 "Q2" V 8025 4075 50  0000 L CNN
F 1 "BSS214NWH6327XTSA1" V 8100 4075 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-323_SC-70_Handsoldering" H 8075 4600 50  0001 L CIN
F 3 "" H 7875 4675 50  0000 L CNN
	1    7875 4675
	1    0    0    -1  
$EndComp
Text GLabel 7675 4300 1    40   Input ~ 0
D10
$Comp
L CONN_01X03 P6
U 1 1 58F61FB9
P 10150 1650
F 0 "P6" H 10150 1850 50  0000 C CNN
F 1 "Pot hdr" V 10250 1650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 10150 1650 50  0001 C CNN
F 3 "" H 10150 1650 50  0000 C CNN
	1    10150 1650
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR025
U 1 1 58F62A82
P 9600 1950
F 0 "#PWR025" H 9600 1700 50  0001 C CNN
F 1 "Earth" H 9600 1800 50  0001 C CNN
F 2 "" H 9600 1950 50  0000 C CNN
F 3 "" H 9600 1950 50  0000 C CNN
	1    9600 1950
	1    0    0    -1  
$EndComp
Text GLabel 9950 1400 1    39   Input ~ 0
3V3
Text GLabel 9950 1850 3    40   Output ~ 0
TXO
$Comp
L SW_SPST SW9
U 1 1 58EFA999
P 7650 1175
F 0 "SW9" H 7350 1175 50  0000 C CNN
F 1 "SW_SPST" H 7650 1100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7650 1175 50  0001 C CNN
F 3 "" H 7650 1175 50  0000 C CNN
	1    7650 1175
	0    1    1    0   
$EndComp
Text GLabel 9450 1800 0    40   Output ~ 0
A7
Text GLabel 9900 2475 1    40   Output ~ 0
VCC
$Comp
L Earth #PWR026
U 1 1 58F6D5C9
P 9900 2900
F 0 "#PWR026" H 9900 2650 50  0001 C CNN
F 1 "Earth" H 9900 2750 50  0001 C CNN
F 2 "" H 9900 2900 50  0000 C CNN
F 3 "" H 9900 2900 50  0000 C CNN
	1    9900 2900
	1    0    0    -1  
$EndComp
$Comp
L R_Small R5
U 1 1 58F72777
P 4875 7275
F 0 "R5" H 4725 7225 50  0000 L CNN
F 1 "10K" H 4675 7325 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 4875 7275 50  0001 C CNN
F 3 "" H 4875 7275 50  0000 C CNN
	1    4875 7275
	1    0    0    -1  
$EndComp
$Comp
L R_Small R4
U 1 1 58F72D01
P 4625 7275
F 0 "R4" H 4475 7225 50  0000 L CNN
F 1 "10K" H 4425 7325 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 4625 7275 50  0001 C CNN
F 3 "" H 4625 7275 50  0000 C CNN
	1    4625 7275
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR027
U 1 1 58F73323
P 4625 7375
F 0 "#PWR027" H 4625 7125 50  0001 C CNN
F 1 "Earth" H 4625 7225 50  0001 C CNN
F 2 "" H 4625 7375 50  0000 C CNN
F 3 "" H 4625 7375 50  0000 C CNN
	1    4625 7375
	1    0    0    -1  
$EndComp
Text GLabel 4875 7375 3    40   Input ~ 0
3V3
$Comp
L SW_SPST SW3
U 1 1 58F83CEF
P 5175 1800
F 0 "SW3" H 4900 1850 50  0000 C CNN
F 1 "SW_SPST" H 5200 1725 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 5175 1800 50  0001 C CNN
F 3 "" H 5175 1800 50  0000 C CNN
	1    5175 1800
	0    1    1    0   
$EndComp
$Comp
L SW_SPST SW6
U 1 1 58F8A53D
P 7150 1700
F 0 "SW6" H 7050 1625 50  0000 C CNN
F 1 "SW_SPST" H 7475 1625 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7150 1700 50  0001 C CNN
F 3 "" H 7150 1700 50  0000 C CNN
	1    7150 1700
	0    1    1    0   
$EndComp
$Comp
L SW_SPST SW5
U 1 1 58F8AB77
P 7000 1175
F 0 "SW5" H 6700 1175 50  0000 C CNN
F 1 "SW_SPST" H 7000 1100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7000 1175 50  0001 C CNN
F 3 "" H 7000 1175 50  0000 C CNN
	1    7000 1175
	0    1    1    0   
$EndComp
$Comp
L SW_SPST SW2
U 1 1 58F8CB7B
P 4875 1800
F 0 "SW2" H 4600 1850 50  0000 C CNN
F 1 "SW_SPST" H 4900 1725 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 4875 1800 50  0001 C CNN
F 3 "" H 4875 1800 50  0000 C CNN
	1    4875 1800
	0    1    1    0   
$EndComp
Text GLabel 8250 2025 1    40   Output ~ 0
D8
Text GLabel 8350 2025 1    40   Output ~ 0
A1
Text GLabel 8450 2025 1    40   Output ~ 0
A2
Text GLabel 8550 2025 1    40   Output ~ 0
A3
Text GLabel 5950 1900 1    40   Output ~ 0
SDA
$Comp
L R_Small R21
U 1 1 58F8EFAE
P 8350 3100
F 0 "R21" V 8400 2875 50  0000 L CNN
F 1 "1M" V 8400 3200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8350 3100 50  0001 C CNN
F 3 "" H 8350 3100 50  0000 C CNN
	1    8350 3100
	-1   0    0    1   
$EndComp
$Comp
L R_Small R23
U 1 1 58F8F07E
P 8450 3100
F 0 "R23" V 8500 2875 50  0000 L CNN
F 1 "1M" V 8500 3200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8450 3100 50  0001 C CNN
F 3 "" H 8450 3100 50  0000 C CNN
	1    8450 3100
	-1   0    0    1   
$EndComp
$Comp
L R_Small R24
U 1 1 58F8F14D
P 8550 3100
F 0 "R24" V 8600 2875 50  0000 L CNN
F 1 "1M" V 8600 3200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 8550 3100 50  0001 C CNN
F 3 "" H 8550 3100 50  0000 C CNN
	1    8550 3100
	-1   0    0    1   
$EndComp
$Comp
L SW_SPDT SW4
U 1 1 58F1249D
P 5575 1750
F 0 "SW4" H 5350 1875 50  0000 C CNN
F 1 "SW_SPDT" H 5650 1575 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 5575 1750 50  0001 C CNN
F 3 "" H 5575 1750 50  0001 C CNN
	1    5575 1750
	0    1    1    0   
$EndComp
$Comp
L R_Small R7
U 1 1 58F12A45
P 6050 2650
F 0 "R7" V 6100 2475 50  0000 L CNN
F 1 "1M" V 6100 2775 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 6050 2650 50  0001 C CNN
F 3 "" H 6050 2650 50  0000 C CNN
	1    6050 2650
	-1   0    0    1   
$EndComp
$Comp
L R_Small R8
U 1 1 58F12BC3
P 6150 2650
F 0 "R8" V 6200 2475 50  0000 L CNN
F 1 "1M" V 6200 2775 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 6150 2650 50  0001 C CNN
F 3 "" H 6150 2650 50  0000 C CNN
	1    6150 2650
	-1   0    0    1   
$EndComp
$Comp
L R_Small R9
U 1 1 58F12D22
P 6250 2650
F 0 "R9" V 6300 2475 50  0000 L CNN
F 1 "1M" V 6300 2775 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 6250 2650 50  0001 C CNN
F 3 "" H 6250 2650 50  0000 C CNN
	1    6250 2650
	-1   0    0    1   
$EndComp
Text GLabel 5925 1350 1    39   Input ~ 0
3V3
Text GLabel 6250 1900 1    40   Output ~ 0
D7
Text GLabel 6150 1900 1    40   Output ~ 0
D6
Text GLabel 6050 1900 1    40   Output ~ 0
SCL
$Comp
L Earth #PWR028
U 1 1 58F17F76
P 5775 3025
F 0 "#PWR028" H 5775 2775 50  0001 C CNN
F 1 "Earth" H 5775 2875 50  0001 C CNN
F 2 "" H 5775 3025 50  0000 C CNN
F 3 "" H 5775 3025 50  0000 C CNN
	1    5775 3025
	1    0    0    -1  
$EndComp
$Comp
L R_Small R6
U 1 1 58F1BC44
P 5950 2650
F 0 "R6" V 6000 2475 50  0000 L CNN
F 1 "1M" V 6000 2775 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 5950 2650 50  0001 C CNN
F 3 "" H 5950 2650 50  0000 C CNN
	1    5950 2650
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG029
U 1 1 58F27EB0
P 9725 2875
F 0 "#FLG029" H 9725 2970 50  0001 C CNN
F 1 "PWR_FLAG" V 9625 2975 50  0000 C CNN
F 2 "" H 9725 2875 50  0000 C CNN
F 3 "" H 9725 2875 50  0000 C CNN
	1    9725 2875
	0    -1   -1   0   
$EndComp
$Comp
L PWR_FLAG #FLG030
U 1 1 58F2895D
P 9725 2525
F 0 "#FLG030" H 9725 2620 50  0001 C CNN
F 1 "PWR_FLAG" V 9625 2625 50  0000 C CNN
F 2 "" H 9725 2525 50  0000 C CNN
F 3 "" H 9725 2525 50  0000 C CNN
	1    9725 2525
	0    -1   -1   0   
$EndComp
$Comp
L LDD-C514RI AFF1
U 1 1 58F31593
P 9600 4825
F 0 "AFF1" H 9600 5325 50  0000 C CNN
F 1 "LDD-C514RI" H 9600 4375 50  0000 C CNN
F 2 "Displays_7-Segment:DA56" H 9600 4825 50  0001 C CNN
F 3 "" H 9600 4825 50  0001 C CNN
	1    9600 4825
	1    0    0    -1  
$EndComp
NoConn ~ 10450 5125
NoConn ~ 8750 5125
$Comp
L Crystal Y1
U 1 1 58F360F0
P 950 4175
F 0 "Y1" H 950 4325 50  0000 C CNN
F 1 "Crystal" H 950 4025 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_Abracon_ABM3-2pin_5.0x3.2mm_HandSoldering" H 950 4175 50  0001 C CNN
F 3 "" H 950 4175 50  0001 C CNN
	1    950  4175
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR031
U 1 1 58F362CD
P 1175 4675
F 0 "#PWR031" H 1175 4425 50  0001 C CNN
F 1 "Earth" H 1175 4525 50  0001 C CNN
F 2 "" H 1175 4675 50  0000 C CNN
F 3 "" H 1175 4675 50  0000 C CNN
	1    1175 4675
	1    0    0    -1  
$EndComp
$Comp
L C_Small C5
U 1 1 58F36360
P 1175 4450
F 0 "C5" H 1185 4520 50  0000 L CNN
F 1 "18pF" H 1185 4370 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1175 4450 50  0001 C CNN
F 3 "" H 1175 4450 50  0001 C CNN
	1    1175 4450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C2
U 1 1 58F3644F
P 700 4450
F 0 "C2" H 710 4520 50  0000 L CNN
F 1 "18pF" H 710 4370 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 700 4450 50  0001 C CNN
F 3 "" H 700 4450 50  0001 C CNN
	1    700  4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 4775 1375 4775
Wire Wire Line
	1600 4875 1375 4875
Wire Wire Line
	1375 4975 1600 4975
Wire Wire Line
	1375 4775 1375 5150
Connection ~ 1375 4875
Connection ~ 1375 4975
Wire Wire Line
	1375 2775 1600 2775
Wire Wire Line
	1375 2125 1375 2975
Wire Wire Line
	1375 2675 1600 2675
Connection ~ 1375 2675
Wire Wire Line
	8750 4325 7975 4325
Wire Wire Line
	7975 4325 7975 4475
Wire Wire Line
	8700 5025 8700 6050
Wire Wire Line
	8750 4925 8625 4925
Wire Wire Line
	8625 4925 8625 6050
Wire Wire Line
	8750 4825 8550 4825
Wire Wire Line
	8550 4825 8550 6050
Wire Wire Line
	8750 4725 8475 4725
Wire Wire Line
	8475 4725 8475 6050
Wire Wire Line
	8750 4625 8400 4625
Wire Wire Line
	8400 4625 8400 6050
Wire Wire Line
	8750 4525 8325 4525
Wire Wire Line
	8325 4525 8325 6050
Wire Wire Line
	8750 4425 8250 4425
Wire Wire Line
	8250 4425 8250 6050
Wire Wire Line
	10525 5025 10525 5375
Wire Wire Line
	10450 4925 10600 4925
Wire Wire Line
	10600 4925 10600 5450
Wire Wire Line
	10450 4825 10675 4825
Wire Wire Line
	10675 4825 10675 5525
Wire Wire Line
	10450 4725 10750 4725
Wire Wire Line
	10750 4725 10750 5600
Wire Wire Line
	10450 4625 10825 4625
Wire Wire Line
	10825 4625 10825 5675
Wire Wire Line
	10450 4525 10900 4525
Wire Wire Line
	10900 4525 10900 5750
Wire Wire Line
	10450 4425 10975 4425
Wire Wire Line
	10975 4425 10975 5825
Wire Wire Line
	10525 5375 8700 5375
Wire Wire Line
	10600 5450 8625 5450
Connection ~ 8625 5450
Wire Wire Line
	10675 5525 8550 5525
Connection ~ 8550 5525
Wire Wire Line
	10750 5600 8475 5600
Connection ~ 8475 5600
Wire Wire Line
	10825 5675 8400 5675
Connection ~ 8400 5675
Wire Wire Line
	10900 5750 8325 5750
Connection ~ 8325 5750
Wire Wire Line
	10975 5825 8250 5825
Connection ~ 8250 5825
Wire Wire Line
	10450 4325 10450 3975
Wire Wire Line
	10450 3975 6925 3975
Wire Wire Line
	6925 3975 6925 4475
Connection ~ 8700 5375
Wire Wire Line
	1250 2975 1600 2975
Connection ~ 1375 2775
Wire Wire Line
	1250 3275 1600 3275
Wire Wire Line
	4275 6475 5025 6475
Wire Wire Line
	4900 6625 5025 6625
Wire Wire Line
	4900 5975 4900 6625
Wire Wire Line
	4275 5975 4900 5975
Wire Wire Line
	4600 5875 4600 6100
Wire Wire Line
	4275 5975 4275 6100
Connection ~ 4600 5975
Wire Wire Line
	4275 6300 4275 6475
Wire Wire Line
	4600 6300 4600 6575
Connection ~ 4600 6475
Wire Wire Line
	1475 3275 1475 3350
Connection ~ 1475 3275
Connection ~ 1375 2975
Connection ~ 2700 6125
Connection ~ 2700 5925
Connection ~ 2700 6325
Wire Wire Line
	2700 6775 2700 6325
Wire Wire Line
	2100 6775 2700 6775
Wire Wire Line
	850  6775 1900 6775
Wire Wire Line
	850  6325 1350 6325
Wire Wire Line
	2700 5925 3200 5925
Wire Wire Line
	1350 6775 1350 6625
Wire Wire Line
	1350 6325 1350 6425
Wire Wire Line
	1150 6425 1150 6325
Wire Wire Line
	1150 6625 1150 7425
Connection ~ 1150 6325
Connection ~ 1150 6775
Connection ~ 1350 6325
Connection ~ 1350 6775
Wire Wire Line
	1150 7425 800  7425
Wire Wire Line
	800  7125 1550 7125
Connection ~ 1150 7125
Wire Wire Line
	1300 7125 1300 7525
Wire Wire Line
	1300 7525 1550 7525
Connection ~ 1300 7125
Wire Wire Line
	1750 7125 2000 7125
Wire Wire Line
	1750 7525 2000 7525
Wire Wire Line
	1300 7325 1550 7325
Connection ~ 1300 7325
Wire Wire Line
	1750 7325 2000 7325
Wire Wire Line
	3200 6325 3075 6325
Wire Wire Line
	2700 6325 2875 6325
Wire Wire Line
	6625 4650 6625 4725
Wire Wire Line
	7675 4650 7675 4725
Wire Wire Line
	700  3925 700  4350
Wire Wire Line
	1175 3925 1175 4350
Wire Wire Line
	3500 4125 4275 4125
Wire Wire Line
	4200 4050 4200 4275
Connection ~ 4200 4125
Wire Wire Line
	4200 3800 4200 3850
Wire Wire Line
	4475 4125 4525 4125
Wire Wire Line
	675  2650 675  2850
Wire Wire Line
	675  2800 975  2800
Wire Wire Line
	975  2800 975  2650
Connection ~ 675  2800
Wire Wire Line
	675  2450 675  2300
Wire Wire Line
	675  2300 1375 2300
Wire Wire Line
	975  2450 975  2300
Connection ~ 975  2300
Connection ~ 1375 2300
Wire Wire Line
	7800 2125 8050 2125
Wire Wire Line
	7800 2125 7800 1900
Wire Wire Line
	7650 2225 8150 2225
Wire Wire Line
	7650 2225 7650 1375
Wire Wire Line
	7475 1900 7475 2325
Wire Wire Line
	7475 2325 8250 2325
Wire Wire Line
	7000 975  8175 975 
Connection ~ 7800 1500
Wire Wire Line
	8050 2025 8050 3000
Connection ~ 8050 2125
Wire Wire Line
	8150 2025 8150 3000
Connection ~ 8150 2225
Wire Wire Line
	8250 2025 8250 3000
Connection ~ 8250 2325
Wire Wire Line
	7875 3450 7875 3500
Wire Wire Line
	8050 3200 8050 3450
Connection ~ 8050 3450
Wire Wire Line
	8150 3450 8150 3200
Connection ~ 8150 3450
Wire Wire Line
	8250 3450 8250 3200
Wire Wire Line
	8175 975  8175 1500
Wire Wire Line
	8175 1250 8325 1250
Connection ~ 8175 1250
Wire Wire Line
	3900 4250 3900 4125
Connection ~ 3900 4125
Wire Wire Line
	9600 1650 9950 1650
Wire Wire Line
	9950 1550 9950 1400
Wire Wire Line
	9950 1750 9950 1850
Wire Wire Line
	4500 6775 5025 6775
Wire Wire Line
	4500 6925 5025 6925
Wire Wire Line
	4625 7175 4625 6775
Connection ~ 4625 6775
Wire Wire Line
	4875 7175 4875 6925
Connection ~ 4875 6925
Connection ~ 7650 975 
Connection ~ 7475 1500
Connection ~ 7325 975 
Wire Wire Line
	7325 1375 7325 2525
Wire Wire Line
	7325 2525 8350 2525
Wire Wire Line
	7150 1900 7150 2625
Wire Wire Line
	7150 2625 8450 2625
Wire Wire Line
	7000 1375 7000 2725
Wire Wire Line
	7000 2725 8550 2725
Connection ~ 7150 1500
Wire Wire Line
	8350 2025 8350 3000
Connection ~ 8350 2525
Wire Wire Line
	8450 2025 8450 3000
Connection ~ 8450 2625
Wire Wire Line
	8550 2025 8550 3000
Connection ~ 8550 2725
Connection ~ 8250 3450
Wire Wire Line
	8350 3450 8350 3200
Connection ~ 8350 3450
Wire Wire Line
	8450 3450 8450 3200
Connection ~ 8450 3450
Wire Wire Line
	8550 3450 8550 3200
Connection ~ 8550 3450
Wire Wire Line
	6250 2975 6250 2750
Wire Wire Line
	5775 2975 6250 2975
Wire Wire Line
	6150 2750 6150 2975
Connection ~ 6150 2975
Wire Wire Line
	6050 2750 6050 2975
Connection ~ 6050 2975
Wire Wire Line
	6250 1900 6250 2550
Wire Wire Line
	6150 1900 6150 2550
Wire Wire Line
	6050 1900 6050 2550
Wire Wire Line
	5175 1600 5175 1425
Wire Wire Line
	4875 1425 5925 1425
Wire Wire Line
	5575 1550 5575 1425
Connection ~ 5575 1425
Connection ~ 6250 2075
Wire Wire Line
	5475 2175 6150 2175
Connection ~ 6150 2175
Connection ~ 6050 2275
Wire Wire Line
	8175 1500 7150 1500
Wire Wire Line
	7875 3450 8550 3450
Wire Wire Line
	4875 1425 4875 1600
Connection ~ 5175 1425
Wire Wire Line
	5950 1900 5950 2550
Wire Wire Line
	5950 2750 5950 2975
Connection ~ 5950 2975
Connection ~ 5950 2375
Wire Wire Line
	5475 1950 5475 2175
Wire Wire Line
	5675 1950 5675 2075
Wire Wire Line
	5675 2075 6250 2075
Wire Wire Line
	5175 2000 5175 2275
Wire Wire Line
	5175 2275 6050 2275
Wire Wire Line
	4875 2000 4875 2375
Wire Wire Line
	4875 2375 5950 2375
Wire Wire Line
	9900 2850 9900 2900
Wire Wire Line
	9725 2875 9900 2875
Connection ~ 9900 2875
Wire Wire Line
	9900 2475 9900 2550
Wire Wire Line
	9900 2525 9725 2525
Connection ~ 9900 2525
Wire Wire Line
	8700 5025 8750 5025
Wire Wire Line
	10450 5025 10525 5025
Wire Wire Line
	1175 4175 1100 4175
Wire Wire Line
	700  4175 800  4175
Connection ~ 700  4175
Wire Wire Line
	700  4550 700  4675
Wire Wire Line
	1175 4675 1175 4550
Connection ~ 1175 4175
Wire Wire Line
	5925 1425 5925 1350
Wire Wire Line
	8325 1250 8325 1150
Wire Wire Line
	6625 4300 6625 4450
Wire Wire Line
	6925 4875 6925 5000
Wire Wire Line
	6350 4375 6625 4375
Wire Wire Line
	6350 4950 6925 4950
Connection ~ 6925 4950
Connection ~ 6625 4375
Wire Wire Line
	7675 4300 7675 4450
Wire Wire Line
	7375 4375 7675 4375
Connection ~ 7675 4375
Wire Wire Line
	7975 4875 7975 5000
Wire Wire Line
	7375 4950 7975 4950
Connection ~ 7975 4950
Wire Wire Line
	5775 3025 5775 2975
$Comp
L R_Small R25
U 1 1 58F8A9E2
P 6350 4550
F 0 "R25" H 6150 4600 50  0000 L CNN
F 1 "10K" H 6150 4500 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 6350 4550 50  0001 C CNN
F 3 "" H 6350 4550 50  0000 C CNN
	1    6350 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 4375 6350 4450
Wire Wire Line
	6350 4650 6350 4950
$Comp
L R_Small R26
U 1 1 58F8B0E5
P 7375 4550
F 0 "R26" H 7175 4600 50  0000 L CNN
F 1 "10K" H 7175 4500 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" H 7375 4550 50  0001 C CNN
F 3 "" H 7375 4550 50  0000 C CNN
	1    7375 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7375 4375 7375 4450
Wire Wire Line
	7375 4650 7375 4950
Text GLabel 3200 6325 2    40   Input ~ 0
VCC
Text GLabel 3200 5925 2    40   Input ~ 0
VCC
$Comp
L SW_SPST SW7
U 1 1 58F8A1B0
P 7325 1175
F 0 "SW7" H 7025 1175 50  0000 C CNN
F 1 "SW_SPST" H 7325 1100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7325 1175 50  0001 C CNN
F 3 "" H 7325 1175 50  0000 C CNN
	1    7325 1175
	0    1    1    0   
$EndComp
$EndSCHEMATC
