EESchema Schematic File Version 2
LIBS:Libreria_SCH_mia
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
LIBS:Microduino_RTC_DE-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Microduino RTC DE"
Date "11 ottobre 2017"
Rev "0.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L PCF8563 U1
U 1 1 58B7F974
P 3400 3800
F 0 "U1" H 3100 4150 50  0000 L CNN
F 1 "PCF8563" H 3500 3400 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3400 3800 50  0001 C CNN
F 3 "" H 3400 3800 50  0000 C CNN
	1    3400 3800
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 58B7FA73
P 2800 3800
F 0 "Y1" V 2800 3950 50  0000 C CNN
F 1 "32,762 kHz" H 2800 3650 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_G8-2pin_3.2x1.5mm_HandSoldering" H 2800 3800 50  0001 C CNN
F 3 "" H 2800 3800 50  0000 C CNN
	1    2800 3800
	0    1    -1   0   
$EndComp
$Comp
L C_Variable CV1
U 1 1 58B7FAE6
P 2800 3250
F 0 "CV1" H 2825 3175 50  0000 L CNN
F 1 "5-25 pF" H 2825 3100 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 2800 3250 50  0001 C CNN
F 3 "" H 2800 3250 50  0000 C CNN
	1    2800 3250
	-1   0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 58B7FB48
P 3800 3100
F 0 "C1" H 3810 3170 50  0000 L CNN
F 1 "0,1 uF" H 3810 3020 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3800 3100 50  0001 C CNN
F 3 "" H 3800 3100 50  0000 C CNN
	1    3800 3100
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 58B7FC81
P 3400 4400
F 0 "#PWR01" H 3400 4150 50  0001 C CNN
F 1 "GND" H 3400 4250 50  0000 C CNN
F 2 "" H 3400 4400 50  0000 C CNN
F 3 "" H 3400 4400 50  0000 C CNN
	1    3400 4400
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 58C44D2B
P 4750 3200
F 0 "R1" H 4850 3200 50  0000 C CNN
F 1 "4k7" V 4750 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4680 3200 50  0001 C CNN
F 3 "" H 4750 3200 50  0000 C CNN
	1    4750 3200
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 58C44D52
P 5000 3200
F 0 "R2" H 5100 3200 50  0000 C CNN
F 1 "4k7" V 5000 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4930 3200 50  0001 C CNN
F 3 "" H 5000 3200 50  0000 C CNN
	1    5000 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 58B7FE98
P 3800 3250
F 0 "#PWR02" H 3800 3000 50  0001 C CNN
F 1 "GND" H 3800 3100 50  0000 C CNN
F 2 "" H 3800 3250 50  0000 C CNN
F 3 "" H 3800 3250 50  0000 C CNN
	1    3800 3250
	1    0    0    -1  
$EndComp
$Comp
L CP C2
U 1 1 58C58191
P 2150 3650
F 0 "C2" H 2175 3750 50  0000 L CNN
F 1 "0,2F" H 2175 3550 50  0000 L CNN
F 2 "Microduino_RTC_DE:Supercap" H 2188 3500 50  0001 C CNN
F 3 "DSK-3R3H204T614-H2L" H 2150 3650 50  0001 C CNN
	1    2150 3650
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D1
U 1 1 58C5822F
P 4500 2900
F 0 "D1" H 4500 3000 50  0000 C CNN
F 1 "TMMBAT43" H 4500 2800 50  0000 C CNN
F 2 "Diodes_SMD:D_MiniMELF_Standard" H 4500 2900 50  0001 C CNN
F 3 "" H 4500 2900 50  0000 C CNN
	1    4500 2900
	1    0    0    1   
$EndComp
NoConn ~ 9400 2850
NoConn ~ 9500 2850
NoConn ~ 9100 2850
NoConn ~ 9000 2850
NoConn ~ 8900 2850
NoConn ~ 8800 2850
NoConn ~ 8700 2850
NoConn ~ 10150 3400
NoConn ~ 10150 3500
NoConn ~ 10150 3800
NoConn ~ 10150 3900
NoConn ~ 10150 4100
$Comp
L GND #PWR03
U 1 1 58D4FFFC
P 9800 4350
F 0 "#PWR03" H 9800 4100 50  0001 C CNN
F 1 "GND" H 9800 4200 50  0000 C CNN
F 2 "" H 9800 4350 50  0000 C CNN
F 3 "" H 9800 4350 50  0000 C CNN
	1    9800 4350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR04
U 1 1 58D50BEE
P 7750 4200
F 0 "#PWR04" H 7750 4050 50  0001 C CNN
F 1 "+5V" H 7750 4340 50  0000 C CNN
F 2 "" H 7750 4200 50  0000 C CNN
F 3 "" H 7750 4200 50  0000 C CNN
	1    7750 4200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 58D50CA7
P 7950 4100
F 0 "#PWR05" H 7950 3950 50  0001 C CNN
F 1 "+3.3V" H 7950 4240 50  0000 C CNN
F 2 "" H 7950 4100 50  0000 C CNN
F 3 "" H 7950 4100 50  0000 C CNN
	1    7950 4100
	1    0    0    -1  
$EndComp
Text Label 8150 4000 0    60   ~ 0
D7
Text Label 8150 3900 0    60   ~ 0
D8
Text Label 8150 3800 0    60   ~ 0
D9
Text Label 8150 3700 0    60   ~ 0
D10
Text Label 8150 3600 0    60   ~ 0
D11
Text Label 8150 3500 0    60   ~ 0
D12
Text Label 8150 3400 0    60   ~ 0
D13
$Comp
L +5V #PWR06
U 1 1 58D512B2
P 6550 2800
F 0 "#PWR06" H 6550 2650 50  0001 C CNN
F 1 "+5V" H 6550 2940 50  0000 C CNN
F 2 "" H 6550 2800 50  0000 C CNN
F 3 "" H 6550 2800 50  0000 C CNN
	1    6550 2800
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 58D512DE
P 6550 3000
F 0 "#PWR07" H 6550 2850 50  0001 C CNN
F 1 "+3.3V" H 6550 3140 50  0000 C CNN
F 2 "" H 6550 3000 50  0000 C CNN
F 3 "" H 6550 3000 50  0000 C CNN
	1    6550 3000
	-1   0    0    1   
$EndComp
NoConn ~ 8150 4000
NoConn ~ 8150 3900
NoConn ~ 8150 3800
NoConn ~ 8150 3700
NoConn ~ 8150 3600
NoConn ~ 8150 3500
NoConn ~ 8150 3400
Text Label 8700 2850 3    60   ~ 0
AREF
Text Label 8800 2850 3    60   ~ 0
A0
Text Label 8900 2850 3    60   ~ 0
A1
Text Label 9000 2850 3    60   ~ 0
A2
Text Label 9100 2850 3    60   ~ 0
A3
Text Label 9200 2850 3    60   ~ 0
SDA
Text Label 9300 2850 3    60   ~ 0
SCL
Text Label 9400 2850 3    60   ~ 0
A6
Text Label 9500 2850 3    60   ~ 0
A7
Wire Wire Line
	3400 4200 3400 4400
Wire Wire Line
	3000 3600 2800 3600
Wire Wire Line
	3000 4000 2800 4000
Wire Wire Line
	3800 2900 3800 3000
Wire Wire Line
	3400 2900 3400 3400
Wire Wire Line
	3800 3200 3800 3250
Wire Wire Line
	3800 3600 5300 3600
Wire Wire Line
	3800 3700 5300 3700
Wire Wire Line
	5000 2900 5000 3050
Connection ~ 3800 2900
Wire Wire Line
	4750 2900 4750 3050
Connection ~ 4750 2900
Wire Wire Line
	4750 3350 4750 3600
Connection ~ 4750 3600
Wire Wire Line
	5000 3350 5000 3700
Connection ~ 5000 3700
Wire Wire Line
	1500 2900 3950 2900
Wire Wire Line
	4650 2900 6150 2900
Wire Wire Line
	2800 2900 2800 3100
Connection ~ 3400 2900
Wire Wire Line
	2800 3400 2800 3650
Wire Wire Line
	2800 4000 2800 3950
Wire Wire Line
	2150 3500 2150 2900
Connection ~ 2800 2900
Wire Wire Line
	2150 3800 2150 4350
Wire Wire Line
	1500 4350 3400 4350
Connection ~ 3400 4350
Connection ~ 5000 2900
Wire Wire Line
	8150 3500 8400 3500
Wire Wire Line
	8400 3700 8150 3700
Wire Wire Line
	8400 3900 8150 3900
Wire Wire Line
	8400 4100 7950 4100
Wire Wire Line
	10150 3500 9800 3500
Wire Wire Line
	10150 3700 9800 3700
Wire Wire Line
	10150 3900 9800 3900
Wire Wire Line
	10150 4100 9800 4100
Wire Wire Line
	6550 2800 6450 2800
Wire Wire Line
	6450 3000 6550 3000
Connection ~ 2800 3600
Wire Wire Line
	9800 4200 9800 4350
Wire Wire Line
	7750 4200 8400 4200
Wire Wire Line
	8400 4000 8150 4000
Wire Wire Line
	8400 3400 8150 3400
Wire Wire Line
	8400 3600 8150 3600
Wire Wire Line
	8400 3800 8150 3800
Wire Wire Line
	9500 3100 9500 2850
Wire Wire Line
	9300 2850 9300 3100
Wire Wire Line
	9100 2850 9100 3100
Wire Wire Line
	8900 2850 8900 3100
Wire Wire Line
	8700 2850 8700 3100
Wire Wire Line
	9400 2850 9400 3100
Wire Wire Line
	9200 2850 9200 3100
Wire Wire Line
	9000 2850 9000 3100
Wire Wire Line
	8800 2850 8800 3100
NoConn ~ 9300 2850
NoConn ~ 9200 2850
Text Label 5300 3600 2    60   ~ 0
SCL
Text Label 5300 3700 2    60   ~ 0
SDA
Text Label 5000 3900 2    60   ~ 0
CLKO
Text Label 5000 4100 2    60   ~ 0
INT
$Comp
L PWR_FLAG #FLG08
U 1 1 58D52798
P 2150 4350
F 0 "#FLG08" H 2150 4445 50  0001 C CNN
F 1 "PWR_FLAG" H 2150 4530 50  0000 C CNN
F 2 "" H 2150 4350 50  0000 C CNN
F 3 "" H 2150 4350 50  0000 C CNN
	1    2150 4350
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG09
U 1 1 58D527C4
P 2150 2900
F 0 "#FLG09" H 2150 2995 50  0001 C CNN
F 1 "PWR_FLAG" H 2150 3080 50  0000 C CNN
F 2 "" H 2150 2900 50  0000 C CNN
F 3 "" H 2150 2900 50  0000 C CNN
	1    2150 2900
	1    0    0    -1  
$EndComp
Text Label 10150 3400 2    60   ~ 0
RX0
Text Label 10150 3500 2    60   ~ 0
TX0
Text Label 10150 3600 2    60   ~ 0
D2
Text Label 10150 3700 2    60   ~ 0
D3
Text Label 10150 3800 2    60   ~ 0
D4
Text Label 10150 3900 2    60   ~ 0
D5
Text Label 10150 4000 2    60   ~ 0
D6
Text Label 10150 4100 2    60   ~ 0
RESET
Wire Wire Line
	9800 3400 10150 3400
Wire Wire Line
	10150 3600 9800 3600
Wire Wire Line
	9800 3800 10150 3800
Wire Wire Line
	10150 4000 9800 4000
$Comp
L GS3 GS1
U 1 1 58D9FACF
P 6300 2900
F 0 "GS1" H 6300 3150 50  0000 C CNN
F 1 "GS3" H 6350 2701 50  0001 C CNN
F 2 "Connectors:GS3" V 6388 2826 50  0001 C CNN
F 3 "" H 6300 2900 50  0000 C CNN
	1    6300 2900
	-1   0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 58DA426D
P 5700 3450
F 0 "R3" H 5800 3450 50  0000 C CNN
F 1 "4k7" V 5700 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5630 3450 50  0001 C CNN
F 3 "" H 5700 3450 50  0000 C CNN
	1    5700 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 2900 5700 3300
Connection ~ 5700 2900
Wire Wire Line
	5700 3600 5700 4000
Connection ~ 5700 4000
Text Label 6550 3900 0    60   ~ 0
D2
Text Label 6550 4100 0    60   ~ 0
D3
Wire Wire Line
	6550 4100 6450 4100
Wire Wire Line
	6550 3900 6450 3900
$Comp
L GS3 GS2
U 1 1 58D9FB5B
P 5150 4000
F 0 "GS2" H 5150 3750 50  0000 C CNN
F 1 "GS3" H 5200 3801 50  0001 C CNN
F 2 "Connectors:GS3" V 5238 3926 50  0001 C CNN
F 3 "" H 5150 4000 50  0000 C CNN
	1    5150 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4000 6150 4000
Wire Wire Line
	5000 3900 3800 3900
Wire Wire Line
	4400 4100 5000 4100
$Comp
L D_Zener D2
U 1 1 58E73E8C
P 1500 3650
F 0 "D2" V 1500 3750 50  0000 C CNN
F 1 "MM3Z3V3ST1G" H 1500 3550 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-323" H 1500 3650 50  0001 C CNN
F 3 "3,3V" H 1500 3650 50  0001 C CNN
	1    1500 3650
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 58E74C0B
P 4100 2900
F 0 "R4" V 4180 2900 50  0000 C CNN
F 1 "33" V 4100 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4030 2900 50  0001 C CNN
F 3 "" H 4100 2900 50  0000 C CNN
	1    4100 2900
	0    1    -1   0   
$EndComp
Wire Wire Line
	4350 2900 4250 2900
Wire Wire Line
	1500 3500 1500 2900
Connection ~ 2150 2900
Wire Wire Line
	1500 3800 1500 4350
Connection ~ 2150 4350
Wire Wire Line
	4400 4100 4400 4000
Wire Wire Line
	4400 4000 3800 4000
Text Notes 8750 4600 0    118  ~ 24
UPIN 27
$Comp
L CONN_1x27 P1
U 1 1 58E8E495
P 8600 4200
F 0 "P1" H 8600 4100 50  0000 C CNN
F 1 "CONN_1x27" V 9500 4600 50  0001 C CNN
F 2 "Libreria_PCB_mia:Upin_27" H 9600 4600 50  0001 C CNN
F 3 "" H 9600 4600 50  0000 C CNN
	1    8600 4200
	1    0    0    -1  
$EndComp
Text Notes 7100 6950 0    236  Italic 47
DigitEco s.r.l.
$Comp
L GS4 GS3
U 1 1 59DEFC44
P 6300 4000
F 0 "GS3" H 6300 4250 50  0000 C CNN
F 1 "GS4" H 6300 3700 50  0001 C CNN
F 2 "Libreria_PCB_mia:GS4" V 6400 3900 50  0001 C CNN
F 3 "" H 6300 4000 50  0000 C CNN
	1    6300 4000
	-1   0    0    -1  
$EndComp
Text Label 6550 4200 0    60   ~ 0
D6
Wire Wire Line
	6450 4200 6550 4200
$EndSCHEMATC
