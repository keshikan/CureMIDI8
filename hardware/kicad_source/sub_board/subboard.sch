EESchema Schematic File Version 4
LIBS:midi-cache
EELAYER 29 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "\"CureMIDI8\" MIDI-IN board"
Date "2019-01-14"
Rev "A1"
Comp "keshikan (@keshinomi 88pro)"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L subboard:DIN_5-subboard CN1
U 1 1 598C565F
P 2600 1250
F 0 "CN1" H 2600 850 50  0000 C CNN
F 1 "DIN_5" H 2600 1100 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 2600 1250 50  0001 C CNN
F 3 "" H 2600 1250 50  0000 C CNN
	1    2600 1250
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 598C5661
P 3000 1650
F 0 "R1" H 3100 1650 50  0000 C CNN
F 1 "220" V 3000 1650 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 2930 1650 50  0001 C CNN
F 3 "" H 3000 1650 50  0000 C CNN
	1    3000 1650
	-1   0    0    -1  
$EndComp
$Comp
L Device:D_Small D1
U 1 1 598C5662
P 3200 1950
F 0 "D1" V 3200 2000 50  0000 L CNN
F 1 "D_Small" H 3050 1870 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 3200 1950 50  0001 C CNN
F 3 "" V 3200 1950 50  0000 C CNN
	1    3200 1950
	0    -1   1    0   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 598C5663
P 4250 2100
F 0 "C1" H 4000 2100 50  0000 L CNN
F 1 "0.1μF" H 4000 2000 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 4250 2100 50  0001 C CNN
F 3 "" H 4250 2100 50  0000 C CNN
	1    4250 2100
	-1   0    0    -1  
$EndComp
NoConn ~ 2200 1250
NoConn ~ 2600 1650
NoConn ~ 3000 1250
$Comp
L subboard:TLP2361-audio_misc U1
U 1 1 598C06AB
P 3600 1950
F 0 "U1" H 3400 2150 50  0000 L CNN
F 1 "TLP2361" H 3600 2150 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 3400 1750 50  0001 L CIN
F 3 "" H 3600 1950 50  0000 L CNN
	1    3600 1950
	1    0    0    -1  
$EndComp
Text Notes 1050 1300 0    79   ~ 0
MIDI IN JACK 1
Connection ~ 4250 1850
Wire Wire Line
	4100 2250 4100 2050
Wire Wire Line
	4250 2200 4250 2250
Connection ~ 3200 2050
Connection ~ 3200 1850
Wire Wire Line
	3000 1850 3200 1850
Wire Wire Line
	3900 1850 4250 1850
Wire Wire Line
	4100 2050 3900 2050
Wire Wire Line
	4100 2250 4250 2250
Connection ~ 4250 2250
Wire Wire Line
	3900 1950 4300 1950
Wire Wire Line
	3000 1450 3000 1500
Wire Wire Line
	3000 1850 3000 1800
Wire Wire Line
	2200 2050 2200 1450
Wire Wire Line
	2200 2050 3200 2050
Wire Wire Line
	4250 1850 4250 2000
Wire Wire Line
	3200 2050 3300 2050
Wire Wire Line
	3200 1850 3300 1850
Wire Wire Line
	7500 3900 7500 3300
Wire Wire Line
	8300 3700 8300 3650
Wire Wire Line
	8300 3300 8300 3350
Wire Wire Line
	9200 3800 9600 3800
Wire Wire Line
	9400 4100 9550 4100
Wire Wire Line
	9400 3900 9200 3900
Wire Wire Line
	9200 3700 9550 3700
Connection ~ 9550 4100
Wire Wire Line
	9550 4050 9550 4100
Wire Wire Line
	9400 4100 9400 3900
Wire Wire Line
	9550 3700 9550 3850
Connection ~ 9550 3700
Wire Wire Line
	7500 2100 7500 1500
Wire Wire Line
	8300 1900 8300 1850
Wire Wire Line
	8300 1500 8300 1550
Wire Wire Line
	9200 2000 9600 2000
Wire Wire Line
	9400 2300 9550 2300
Wire Wire Line
	9400 2100 9200 2100
Wire Wire Line
	9200 1900 9550 1900
Connection ~ 9550 2300
Wire Wire Line
	9550 2250 9550 2300
Wire Wire Line
	9400 2300 9400 2100
Wire Wire Line
	9550 1900 9550 2050
Connection ~ 9550 1900
Text Notes 6350 3150 0    79   ~ 0
MIDI IN JACK 6
Text Notes 6350 1350 0    79   ~ 0
MIDI IN JACK 5
$Comp
L subboard:TLP2361-audio_misc U6
U 1 1 5BEC3E75
P 8900 3800
F 0 "U6" H 8700 4000 50  0000 L CNN
F 1 "TLP2361" H 8900 4000 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 8700 3600 50  0001 L CIN
F 3 "" H 8900 3800 50  0000 L CNN
	1    8900 3800
	1    0    0    -1  
$EndComp
$Comp
L subboard:TLP2361-audio_misc U5
U 1 1 5BEC3E6E
P 8900 2000
F 0 "U5" H 8700 2200 50  0000 L CNN
F 1 "TLP2361" H 8900 2200 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 8700 1800 50  0001 L CIN
F 3 "" H 8900 2000 50  0000 L CNN
	1    8900 2000
	1    0    0    -1  
$EndComp
NoConn ~ 8300 3100
NoConn ~ 7900 3500
NoConn ~ 7500 3100
$Comp
L Device:C_Small C6
U 1 1 5BEC3E5C
P 9550 3950
F 0 "C6" H 9300 3950 50  0000 L CNN
F 1 "0.1μF" H 9300 3850 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 9550 3950 50  0001 C CNN
F 3 "" H 9550 3950 50  0000 C CNN
	1    9550 3950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8500 3900 8600 3900
Wire Wire Line
	7500 3900 8500 3900
Connection ~ 8500 3900
Wire Wire Line
	8500 3700 8600 3700
Wire Wire Line
	8300 3700 8500 3700
Connection ~ 8500 3700
$Comp
L Device:D_Small D6
U 1 1 5BEC3E55
P 8500 3800
F 0 "D6" V 8500 3850 50  0000 L CNN
F 1 "D_Small" H 8350 3720 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 8500 3800 50  0001 C CNN
F 3 "" V 8500 3800 50  0000 C CNN
	1    8500 3800
	0    -1   1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5BEC3E4E
P 8300 3500
F 0 "R6" H 8400 3500 50  0000 C CNN
F 1 "220" V 8300 3500 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 8230 3500 50  0001 C CNN
F 3 "" H 8300 3500 50  0000 C CNN
	1    8300 3500
	-1   0    0    -1  
$EndComp
$Comp
L subboard:DIN_5-subboard CN6
U 1 1 5BEC3E47
P 7900 3100
F 0 "CN6" H 7900 2700 50  0000 C CNN
F 1 "DIN_5" H 7900 2950 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 7900 3100 50  0001 C CNN
F 3 "" H 7900 3100 50  0000 C CNN
	1    7900 3100
	-1   0    0    1   
$EndComp
NoConn ~ 8300 1300
NoConn ~ 7900 1700
NoConn ~ 7500 1300
$Comp
L Device:C_Small C5
U 1 1 5BEC3E37
P 9550 2150
F 0 "C5" H 9300 2150 50  0000 L CNN
F 1 "0.1μF" H 9300 2050 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 9550 2150 50  0001 C CNN
F 3 "" H 9550 2150 50  0000 C CNN
	1    9550 2150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8500 2100 8600 2100
Wire Wire Line
	7500 2100 8500 2100
Connection ~ 8500 2100
Wire Wire Line
	8500 1900 8600 1900
Wire Wire Line
	8300 1900 8500 1900
Connection ~ 8500 1900
$Comp
L Device:D_Small D5
U 1 1 5BEC3E30
P 8500 2000
F 0 "D5" V 8500 2050 50  0000 L CNN
F 1 "D_Small" H 8350 1920 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 8500 2000 50  0001 C CNN
F 3 "" V 8500 2000 50  0000 C CNN
	1    8500 2000
	0    -1   1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5BEC3E29
P 8300 1700
F 0 "R5" H 8400 1700 50  0000 C CNN
F 1 "220" V 8300 1700 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 8230 1700 50  0001 C CNN
F 3 "" H 8300 1700 50  0000 C CNN
	1    8300 1700
	-1   0    0    -1  
$EndComp
$Comp
L subboard:DIN_5-subboard CN5
U 1 1 5BEC3E22
P 7900 1300
F 0 "CN5" H 7900 900 50  0000 C CNN
F 1 "DIN_5" H 7900 1150 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 7900 1300 50  0001 C CNN
F 3 "" H 7900 1300 50  0000 C CNN
	1    7900 1300
	-1   0    0    1   
$EndComp
Wire Wire Line
	2200 7450 2200 6850
Wire Wire Line
	3000 7250 3000 7200
Wire Wire Line
	3000 6850 3000 6900
Wire Wire Line
	3900 7350 4300 7350
Wire Wire Line
	4100 7650 4250 7650
Wire Wire Line
	4100 7450 3900 7450
Wire Wire Line
	3900 7250 4250 7250
Connection ~ 4250 7650
Wire Wire Line
	4250 7600 4250 7650
Wire Wire Line
	4100 7650 4100 7450
Wire Wire Line
	4250 7250 4250 7400
Connection ~ 4250 7250
Wire Wire Line
	2200 5650 2200 5050
Wire Wire Line
	3000 5450 3000 5400
Wire Wire Line
	3000 5050 3000 5100
Wire Wire Line
	3900 5550 4300 5550
Wire Wire Line
	4100 5850 4250 5850
Wire Wire Line
	4100 5650 3900 5650
Wire Wire Line
	3900 5450 4250 5450
Connection ~ 4250 5850
Wire Wire Line
	4250 5800 4250 5850
Wire Wire Line
	4100 5850 4100 5650
Wire Wire Line
	4250 5450 4250 5600
Connection ~ 4250 5450
Text Notes 1050 6700 0    79   ~ 0
MIDI IN JACK 4
Text Notes 1050 4900 0    79   ~ 0
MIDI IN JACK 3
$Comp
L subboard:TLP2361-audio_misc U4
U 1 1 5BEB3AFB
P 3600 7350
F 0 "U4" H 3400 7550 50  0000 L CNN
F 1 "TLP2361" H 3600 7550 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 3400 7150 50  0001 L CIN
F 3 "" H 3600 7350 50  0000 L CNN
	1    3600 7350
	1    0    0    -1  
$EndComp
$Comp
L subboard:TLP2361-audio_misc U3
U 1 1 5BEB3AF5
P 3600 5550
F 0 "U3" H 3400 5750 50  0000 L CNN
F 1 "TLP2361" H 3600 5750 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 3400 5350 50  0001 L CIN
F 3 "" H 3600 5550 50  0000 L CNN
	1    3600 5550
	1    0    0    -1  
$EndComp
NoConn ~ 3000 6650
NoConn ~ 2600 7050
NoConn ~ 2200 6650
$Comp
L Device:C_Small C4
U 1 1 5BEB3AE4
P 4250 7500
F 0 "C4" H 4000 7500 50  0000 L CNN
F 1 "0.1μF" H 4000 7400 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 4250 7500 50  0001 C CNN
F 3 "" H 4250 7500 50  0000 C CNN
	1    4250 7500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3200 7450 3300 7450
Wire Wire Line
	2200 7450 3200 7450
Connection ~ 3200 7450
Wire Wire Line
	3200 7250 3300 7250
Wire Wire Line
	3000 7250 3200 7250
Connection ~ 3200 7250
$Comp
L Device:D_Small D4
U 1 1 5BEB3ADE
P 3200 7350
F 0 "D4" V 3200 7400 50  0000 L CNN
F 1 "D_Small" H 3050 7270 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 3200 7350 50  0001 C CNN
F 3 "" V 3200 7350 50  0000 C CNN
	1    3200 7350
	0    -1   1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5BEB3AD8
P 3000 7050
F 0 "R4" H 3100 7050 50  0000 C CNN
F 1 "220" V 3000 7050 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 2930 7050 50  0001 C CNN
F 3 "" H 3000 7050 50  0000 C CNN
	1    3000 7050
	-1   0    0    -1  
$EndComp
$Comp
L subboard:DIN_5-subboard CN4
U 1 1 5BEB3AD2
P 2600 6650
F 0 "CN4" H 2600 6250 50  0000 C CNN
F 1 "DIN_5" H 2600 6500 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 2600 6650 50  0001 C CNN
F 3 "" H 2600 6650 50  0000 C CNN
	1    2600 6650
	-1   0    0    1   
$EndComp
NoConn ~ 3000 4850
NoConn ~ 2600 5250
NoConn ~ 2200 4850
$Comp
L Device:C_Small C3
U 1 1 5BEB3AC3
P 4250 5700
F 0 "C3" H 4000 5700 50  0000 L CNN
F 1 "0.1μF" H 4000 5600 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 4250 5700 50  0001 C CNN
F 3 "" H 4250 5700 50  0000 C CNN
	1    4250 5700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3200 5650 3300 5650
Wire Wire Line
	2200 5650 3200 5650
Connection ~ 3200 5650
Wire Wire Line
	3200 5450 3300 5450
Wire Wire Line
	3000 5450 3200 5450
Connection ~ 3200 5450
$Comp
L Device:D_Small D3
U 1 1 5BEB3ABD
P 3200 5550
F 0 "D3" V 3200 5600 50  0000 L CNN
F 1 "D_Small" H 3050 5470 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 3200 5550 50  0001 C CNN
F 3 "" V 3200 5550 50  0000 C CNN
	1    3200 5550
	0    -1   1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5BEB3AB7
P 3000 5250
F 0 "R3" H 3100 5250 50  0000 C CNN
F 1 "220" V 3000 5250 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 2930 5250 50  0001 C CNN
F 3 "" H 3000 5250 50  0000 C CNN
	1    3000 5250
	-1   0    0    -1  
$EndComp
$Comp
L subboard:DIN_5-subboard CN3
U 1 1 5BEB3AB1
P 2600 4850
F 0 "CN3" H 2600 4450 50  0000 C CNN
F 1 "DIN_5" H 2600 4700 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 2600 4850 50  0001 C CNN
F 3 "" H 2600 4850 50  0000 C CNN
	1    2600 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	2200 3850 2200 3250
Wire Wire Line
	3000 3650 3000 3600
Wire Wire Line
	3000 3250 3000 3300
Wire Wire Line
	3900 3750 4300 3750
Wire Wire Line
	4100 4050 4250 4050
Wire Wire Line
	4100 3850 3900 3850
Wire Wire Line
	3900 3650 4250 3650
Connection ~ 4250 4050
Wire Wire Line
	4250 4000 4250 4050
Wire Wire Line
	4100 4050 4100 3850
Wire Wire Line
	4250 3650 4250 3800
Connection ~ 4250 3650
Text Notes 1050 3100 0    79   ~ 0
MIDI IN JACK 2
$Comp
L subboard:TLP2361-audio_misc U2
U 1 1 598C248D
P 3600 3750
F 0 "U2" H 3400 3950 50  0000 L CNN
F 1 "TLP2361" H 3600 3950 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 3400 3550 50  0001 L CIN
F 3 "" H 3600 3750 50  0000 L CNN
	1    3600 3750
	1    0    0    -1  
$EndComp
NoConn ~ 3000 3050
NoConn ~ 2600 3450
NoConn ~ 2200 3050
$Comp
L Device:C_Small C2
U 1 1 599A7886
P 4250 3900
F 0 "C2" H 4000 3900 50  0000 L CNN
F 1 "0.1μF" H 4000 3800 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 4250 3900 50  0001 C CNN
F 3 "" H 4250 3900 50  0000 C CNN
	1    4250 3900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3200 3850 3300 3850
Wire Wire Line
	2200 3850 3200 3850
Connection ~ 3200 3850
Wire Wire Line
	3200 3650 3300 3650
Wire Wire Line
	3000 3650 3200 3650
Connection ~ 3200 3650
$Comp
L Device:D_Small D2
U 1 1 599A7880
P 3200 3750
F 0 "D2" V 3200 3800 50  0000 L CNN
F 1 "D_Small" H 3050 3670 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 3200 3750 50  0001 C CNN
F 3 "" V 3200 3750 50  0000 C CNN
	1    3200 3750
	0    -1   1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 599A787A
P 3000 3450
F 0 "R2" H 3100 3450 50  0000 C CNN
F 1 "220" V 3000 3450 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 2930 3450 50  0001 C CNN
F 3 "" H 3000 3450 50  0000 C CNN
	1    3000 3450
	-1   0    0    -1  
$EndComp
$Comp
L subboard:DIN_5-subboard CN2
U 1 1 599A786E
P 2600 3050
F 0 "CN2" H 2600 2650 50  0000 C CNN
F 1 "DIN_5" H 2600 2900 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 2600 3050 50  0001 C CNN
F 3 "" H 2600 3050 50  0000 C CNN
	1    2600 3050
	-1   0    0    1   
$EndComp
Wire Wire Line
	7500 7600 7500 7000
Wire Wire Line
	8300 7400 8300 7350
Wire Wire Line
	8300 7000 8300 7050
Wire Wire Line
	9200 7500 9600 7500
Wire Wire Line
	9400 7800 9550 7800
Wire Wire Line
	9400 7600 9200 7600
Wire Wire Line
	9200 7400 9550 7400
Connection ~ 9550 7800
Wire Wire Line
	9550 7750 9550 7800
Wire Wire Line
	9400 7800 9400 7600
Wire Wire Line
	9550 7400 9550 7550
Connection ~ 9550 7400
Wire Wire Line
	7500 5800 7500 5200
Wire Wire Line
	8300 5600 8300 5550
Wire Wire Line
	8300 5200 8300 5250
Wire Wire Line
	9200 5700 9600 5700
Wire Wire Line
	9400 6000 9550 6000
Wire Wire Line
	9400 5800 9200 5800
Wire Wire Line
	9200 5600 9550 5600
Connection ~ 9550 6000
Wire Wire Line
	9550 5950 9550 6000
Wire Wire Line
	9400 6000 9400 5800
Wire Wire Line
	9550 5600 9550 5750
Connection ~ 9550 5600
Text Notes 6350 6850 0    79   ~ 0
MIDI IN JACK 8
Text Notes 6350 5050 0    79   ~ 0
MIDI IN JACK 7
$Comp
L subboard:TLP2361-audio_misc U8
U 1 1 5BF0D5CF
P 8900 7500
F 0 "U8" H 8700 7700 50  0000 L CNN
F 1 "TLP2361" H 8900 7700 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 8700 7300 50  0001 L CIN
F 3 "" H 8900 7500 50  0000 L CNN
	1    8900 7500
	1    0    0    -1  
$EndComp
$Comp
L subboard:TLP2361-audio_misc U7
U 1 1 5BF0D5D6
P 8900 5700
F 0 "U7" H 8700 5900 50  0000 L CNN
F 1 "TLP2361" H 8900 5900 50  0000 L CNN
F 2 "keshikan:11-4L1S(TLP2361)" H 8700 5500 50  0001 L CIN
F 3 "" H 8900 5700 50  0000 L CNN
	1    8900 5700
	1    0    0    -1  
$EndComp
NoConn ~ 8300 6800
NoConn ~ 7900 7200
NoConn ~ 7500 6800
$Comp
L Device:C_Small C8
U 1 1 5BF0D5E8
P 9550 7650
F 0 "C8" H 9300 7650 50  0000 L CNN
F 1 "0.1μF" H 9300 7550 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 9550 7650 50  0001 C CNN
F 3 "" H 9550 7650 50  0000 C CNN
	1    9550 7650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8500 7600 8600 7600
Wire Wire Line
	7500 7600 8500 7600
Connection ~ 8500 7600
Wire Wire Line
	8500 7400 8600 7400
Wire Wire Line
	8300 7400 8500 7400
Connection ~ 8500 7400
$Comp
L Device:D_Small D8
U 1 1 5BF0D5F5
P 8500 7500
F 0 "D8" V 8500 7550 50  0000 L CNN
F 1 "D_Small" H 8350 7420 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 8500 7500 50  0001 C CNN
F 3 "" V 8500 7500 50  0000 C CNN
	1    8500 7500
	0    -1   1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5BF0D5FC
P 8300 7200
F 0 "R8" H 8400 7200 50  0000 C CNN
F 1 "220" V 8300 7200 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 8230 7200 50  0001 C CNN
F 3 "" H 8300 7200 50  0000 C CNN
	1    8300 7200
	-1   0    0    -1  
$EndComp
$Comp
L subboard:DIN_5-subboard CN8
U 1 1 5BF0D603
P 7900 6800
F 0 "CN8" H 7900 6400 50  0000 C CNN
F 1 "DIN_5" H 7900 6650 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 7900 6800 50  0001 C CNN
F 3 "" H 7900 6800 50  0000 C CNN
	1    7900 6800
	-1   0    0    1   
$EndComp
NoConn ~ 8300 5000
NoConn ~ 7900 5400
NoConn ~ 7500 5000
$Comp
L Device:C_Small C7
U 1 1 5BF0D613
P 9550 5850
F 0 "C7" H 9300 5850 50  0000 L CNN
F 1 "0.1μF" H 9300 5750 50  0000 L CNN
F 2 "keshikan:SMD_0603(1608)" H 9550 5850 50  0001 C CNN
F 3 "" H 9550 5850 50  0000 C CNN
	1    9550 5850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8500 5800 8600 5800
Wire Wire Line
	7500 5800 8500 5800
Connection ~ 8500 5800
Wire Wire Line
	8500 5600 8600 5600
Wire Wire Line
	8300 5600 8500 5600
Connection ~ 8500 5600
$Comp
L Device:D_Small D7
U 1 1 5BF0D620
P 8500 5700
F 0 "D7" V 8500 5750 50  0000 L CNN
F 1 "D_Small" H 8350 5620 50  0001 L CNN
F 2 "keshikan:SMD_0805(2012)_polar" V 8500 5700 50  0001 C CNN
F 3 "" V 8500 5700 50  0000 C CNN
	1    8500 5700
	0    -1   1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5BF0D627
P 8300 5400
F 0 "R7" H 8400 5400 50  0000 C CNN
F 1 "220" V 8300 5400 50  0000 C CNN
F 2 "keshikan:SMD_0603(1608)" V 8230 5400 50  0001 C CNN
F 3 "" H 8300 5400 50  0000 C CNN
	1    8300 5400
	-1   0    0    -1  
$EndComp
$Comp
L subboard:DIN_5-subboard CN7
U 1 1 5BF0D62E
P 7900 5000
F 0 "CN7" H 7900 4600 50  0000 C CNN
F 1 "DIN_5" H 7900 4850 50  0001 C CNN
F 2 "keshikan:DIN_5P_MIDI" H 7900 5000 50  0001 C CNN
F 3 "" H 7900 5000 50  0000 C CNN
	1    7900 5000
	-1   0    0    1   
$EndComp
Text GLabel 4300 1700 2    43   BiDi ~ 0
IN_3V3
Text GLabel 4300 1950 2    43   BiDi ~ 0
IN_1
Wire Wire Line
	4300 1700 4250 1700
Wire Wire Line
	4250 1700 4250 1850
Text GLabel 4300 3450 2    43   BiDi ~ 0
IN_3V3
Text GLabel 4300 5250 2    43   BiDi ~ 0
IN_3V3
Text GLabel 4300 7050 2    43   BiDi ~ 0
IN_3V3
Text GLabel 9600 1700 2    43   BiDi ~ 0
IN_3V3
Text GLabel 9600 3500 2    43   BiDi ~ 0
IN_3V3
Text GLabel 9600 5400 2    43   BiDi ~ 0
IN_3V3
Text GLabel 9600 7200 2    43   BiDi ~ 0
IN_3V3
Text GLabel 4300 2400 2    43   BiDi ~ 0
IN_GND
Text GLabel 4300 4200 2    43   BiDi ~ 0
IN_GND
Text GLabel 4300 6000 2    43   BiDi ~ 0
IN_GND
Text GLabel 4300 7800 2    43   BiDi ~ 0
IN_GND
Text GLabel 9600 2450 2    43   BiDi ~ 0
IN_GND
Text GLabel 9600 4250 2    43   BiDi ~ 0
IN_GND
Text GLabel 9600 6150 2    43   BiDi ~ 0
IN_GND
Text GLabel 9600 7950 2    43   BiDi ~ 0
IN_GND
Wire Wire Line
	4250 2400 4300 2400
Wire Wire Line
	4250 2250 4250 2400
Wire Wire Line
	4300 3450 4250 3450
Wire Wire Line
	4250 3450 4250 3650
Wire Wire Line
	4250 4200 4300 4200
Wire Wire Line
	4250 4050 4250 4200
Wire Wire Line
	4300 5250 4250 5250
Wire Wire Line
	4250 5250 4250 5450
Wire Wire Line
	4250 6000 4300 6000
Wire Wire Line
	4250 5850 4250 6000
Wire Wire Line
	4300 7050 4250 7050
Wire Wire Line
	4250 7050 4250 7250
Wire Wire Line
	4250 7800 4300 7800
Wire Wire Line
	4250 7650 4250 7800
Wire Wire Line
	9600 1700 9550 1700
Wire Wire Line
	9550 1700 9550 1900
Wire Wire Line
	9550 2450 9600 2450
Wire Wire Line
	9550 2300 9550 2450
Wire Wire Line
	9600 3500 9550 3500
Wire Wire Line
	9550 3500 9550 3700
Wire Wire Line
	9550 4250 9600 4250
Wire Wire Line
	9550 4100 9550 4250
Wire Wire Line
	9550 5400 9600 5400
Wire Wire Line
	9550 5400 9550 5600
Wire Wire Line
	9550 6150 9600 6150
Wire Wire Line
	9550 6000 9550 6150
Wire Wire Line
	9600 7200 9550 7200
Wire Wire Line
	9550 7200 9550 7400
Wire Wire Line
	9550 7950 9600 7950
Wire Wire Line
	9550 7800 9550 7950
Text GLabel 4300 3750 2    43   BiDi ~ 0
IN_2
Text GLabel 4300 5550 2    43   BiDi ~ 0
IN_3
Text GLabel 4300 7350 2    43   BiDi ~ 0
IN_4
Text GLabel 9600 2000 2    43   BiDi ~ 0
IN_5
Text GLabel 9600 3800 2    43   BiDi ~ 0
IN_6
Text GLabel 9600 5700 2    43   BiDi ~ 0
IN_7
Text GLabel 9600 7500 2    43   BiDi ~ 0
IN_8
$Comp
L subboard:Conn_01x10-conn J1
U 1 1 5C5D6A45
P 3300 9200
F 0 "J1" H 3380 9193 50  0000 L CNN
F 1 "Conn_01x10" H 3380 9100 50  0000 L CNN
F 2 "Connectors_JST:JST_XH_B10B-XH-A_10x2.50mm_Straight" H 3300 9200 50  0001 C CNN
F 3 "~" H 3300 9200 50  0001 C CNN
	1    3300 9200
	-1   0    0    -1  
$EndComp
Text GLabel 3800 9000 2    43   BiDi ~ 0
IN_1
Text GLabel 3800 9100 2    43   BiDi ~ 0
IN_2
Text GLabel 3800 9200 2    43   BiDi ~ 0
IN_3
Text GLabel 3800 9300 2    43   BiDi ~ 0
IN_4
Text GLabel 3800 9400 2    43   BiDi ~ 0
IN_5
Text GLabel 3800 9500 2    43   BiDi ~ 0
IN_6
Text GLabel 3800 9600 2    43   BiDi ~ 0
IN_7
Text GLabel 3800 9700 2    43   BiDi ~ 0
IN_8
Text GLabel 3800 8800 2    43   BiDi ~ 0
IN_GND
Text GLabel 3800 8900 2    43   BiDi ~ 0
IN_3V3
Wire Wire Line
	3500 8800 3600 8800
Wire Wire Line
	3800 8900 3700 8900
Wire Wire Line
	3500 9000 3800 9000
Wire Wire Line
	3800 9100 3500 9100
Wire Wire Line
	3500 9200 3800 9200
Wire Wire Line
	3800 9300 3500 9300
Wire Wire Line
	3500 9400 3800 9400
Wire Wire Line
	3800 9500 3500 9500
Wire Wire Line
	3500 9600 3800 9600
Wire Wire Line
	3800 9700 3500 9700
$Comp
L subboard:PWR_FLAG-midi #FLG0101
U 1 1 5C9C5808
P 3600 8750
F 0 "#FLG0101" H 3600 8825 50  0001 C CNN
F 1 "PWR_FLAG" H 3600 8926 50  0001 C CNN
F 2 "" H 3600 8750 50  0001 C CNN
F 3 "" H 3600 8750 50  0001 C CNN
	1    3600 8750
	1    0    0    -1  
$EndComp
$Comp
L subboard:PWR_FLAG-midi #FLG0102
U 1 1 5C9C593E
P 3700 8750
F 0 "#FLG0102" H 3700 8825 50  0001 C CNN
F 1 "PWR_FLAG" H 3700 8926 50  0001 C CNN
F 2 "" H 3700 8750 50  0001 C CNN
F 3 "" H 3700 8750 50  0001 C CNN
	1    3700 8750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 8750 3600 8800
Connection ~ 3600 8800
Wire Wire Line
	3600 8800 3800 8800
Wire Wire Line
	3700 8750 3700 8900
Connection ~ 3700 8900
Wire Wire Line
	3700 8900 3500 8900
Text Notes 1050 9100 0    118  ~ 0
From main board ->
Text Notes 650  10950 0    197  ~ 0
MIDI-IN Board
Wire Notes Line
	600  10600 2900 10600
Wire Notes Line
	2900 10600 2900 11050
Wire Notes Line
	2900 11050 600  11050
Wire Notes Line
	600  11050 600  10600
$EndSCHEMATC
