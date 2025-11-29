# 8.6 Register Map

## 8.6.1 STATUS Registers

STATUS Registers lists the memory-mapped registers for the STATUS registers. All register offset addresses not listed in STATUS Registers should be considered as reserved locations and the register contents should not be modified.

Table 8-11. STATUS Registers  

<table><tr><td>Offset</td><td>Acronym</td><td>Register Name</td><td>Section</td></tr><tr><td>0h</td><td>IC_StatusRegister</td><td>IC Status Register</td><td>Section 8.6.1.1</td></tr><tr><td>1h</td><td>StatusRegister_1</td><td>Status Register 1</td><td>Section 8.6.1.2</td></tr><tr><td>2h</td><td>StatusRegister_2</td><td>Status Register 2</td><td>Section 8.6.1.3</td></tr></table>

Complex bit access types are encoded to fit into small table cells. STATUS Access Type Codes shows the codes that are used for access types in this section.

Table 8-12. STATUS Access Type Codes  

<table><tr><td>Access Type</td><td>Code</td><td>Description</td></tr><tr><td colspan="3">Read Type</td></tr><tr><td>R</td><td>R</td><td>Read</td></tr><tr><td>R-0</td><td>R-0</td><td>ReadReturns 0s</td></tr><tr><td colspan="3">Reset or Default Value</td></tr><tr><td>-n</td><td></td><td>Value after reset or the default value</td></tr></table>

### 8.6.1.1 IC_StatusRegister Register (Offset  \(= 0h\)  [Reset  \(= 00h]\)

IC_StatusRegister is shown in IC_StatusRegister Register and described in IC_StatusRegister Register Field Descriptions.

Return to the STATUS Registers.

Figure 8-49. IC_StatusRegister Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>MTR_LOCK</td><td>BK_FLT</td><td>SPI_FLT</td><td>OCP</td><td>NPOR</td><td>OVP</td><td>OT</td><td>FAULT</td></tr><tr><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td></tr></table>

Table 8-13. IC_StatusRegister Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7</td><td>MTR_LOCK</td><td>R</td><td>0h</td><td>Motor Lock Staus Bit
0h = No motor lock is detected
1h = Motor lock is detected</td></tr><tr><td>6</td><td>BK_FLT</td><td>R</td><td>0h</td><td>Buck Fault Bit
0h = No buck regulator fault condition is detected
1h = Buck regulator fault condition is detected</td></tr><tr><td>5</td><td>SPI_FLT</td><td>R</td><td>0h</td><td>SPI Fault Bit
0h = No SPI fault condition is detected
1h = SPI Fault condition is detected</td></tr><tr><td>4</td><td>OCP</td><td>R</td><td>0h</td><td>Over Current Protection Status Bit
0h = No overcurrent condition is detected
1h = Overcurrent condition is detected</td></tr><tr><td>3</td><td>NPOR</td><td>R</td><td>0h</td><td>Supply Power On Reset Bit
0h = Power on reset condition is detected on VM
1h = No power-on-reset condition is detected on VM</td></tr><tr><td>2</td><td>OVP</td><td>R</td><td>0h</td><td>Supply Overvoltage Protection Status Bit
0h = No overvoltage condition is detected on VM
1h = Overvoltage condition is detected on VM</td></tr><tr><td>1</td><td>OT</td><td>R</td><td>0h</td><td>Overtemperature Fault Status Bit
0h = No overtemperature warning / shutdown is detected
1h = Overtemperature warning / shutdown is detected</td></tr><tr><td>0</td><td>FAULT</td><td>R</td><td>0h</td><td>Device Fault Bit
0h = No fault condition is detected
1h = Fault condition is detected</td></tr></table>

### 8.6.1.2 StatusRegister_1 Register (Offset  \(= 1\mathrm{h}\)  [Reset  \(= 00\mathrm{h}]\)

StatusRegister_1 is shown in StatusRegister_1 Register and described in StatusRegister_1 Register Field Descriptions.

Return to the STATUS Registers.

Figure 8-50. StatusRegister_1 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>OTW</td><td>OTS</td><td>OCP_HC</td><td>OCL_LC</td><td>OCP_HB</td><td>OCP_LB</td><td>OCP_HA</td><td>OCP_LA</td></tr><tr><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td><td>R-Oh</td></tr></table>

Table 8-14. StatusRegister_1 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7</td><td>OTW</td><td>R</td><td>0h</td><td>Overtemperature Warning Status Bit
0h = No overtemperature warning is detected
1h = Overtemperature warning is detected</td></tr><tr><td>6</td><td>OTS</td><td>R</td><td>0h</td><td>Overtemperature Shutdown Status Bit
0h = No overtemperature shutdown is detected
1h = Overtemperature shutdown is detected</td></tr><tr><td>5</td><td>OCP_HC</td><td>R</td><td>0h</td><td>Overcurrent Status on High-side switch of OUTC
0h = No overcurrent detected on high-side switch of OUTC
1h = Overcurrent detected on high-side switch of OUTC</td></tr><tr><td>4</td><td>OCL_LC</td><td>R</td><td>0h</td><td>Overcurrent Status on Low-side switch of OUTC
0h = No overcurrent detected on low-side switch of OUTC
1h = Overcurrent detected on low-side switch of OUTC</td></tr><tr><td>3</td><td>OCP_HB</td><td>R</td><td>0h</td><td>Overcurrent Status on High-side switch of OUTB
0h = No overcurrent detected on high-side switch of OUTB
1h = Overcurrent detected on high-side switch of OUTB</td></tr><tr><td>2</td><td>OCP_LB</td><td>R</td><td>0h</td><td>Overcurrent Status on Low-side switch of OUTB
0h = No overcurrent detected on low-side switch of OUTB
1h = Overcurrent detected on low-side switch of OUTB</td></tr><tr><td>1</td><td>OCP_HA</td><td>R</td><td>0h</td><td>Overcurrent Status on High-side switch of OUTA
0h = No overcurrent detected on high-side switch of OUTA
1h = Overcurrent detected on high-side switch of OUTA</td></tr><tr><td>0</td><td>OCP_LA</td><td>R</td><td>0h</td><td>Overcurrent Status on Low-side switch of OUTA
0h = No overcurrent detected on low-side switch of OUTA
1h = Overcurrent detected on low-side switch of OUTA</td></tr></table>

### 8.6.1.3 StatusRegister_2 Register (Offset = 2h) [Reset = 00h]

StatusRegister_2 is shown in StatusRegister_2 Register and described in StatusRegister_2 Register Field Descriptions.

Return to the STATUS Registers.

Figure 8-51. StatusRegister_2 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>RESERVED</td><td>OTP_ERR</td><td>BUCK_OCP</td><td>BUCK_UV</td><td>VCP_UV</td><td>SPI_PARITY</td><td>SPI_SCLK_FLT</td><td>SPI_ADDR_FL T</td></tr><tr><td>R-0-0h</td><td>R-0h</td><td>R-0h</td><td>R-0h</td><td>R-0h</td><td>R-0-0h</td><td>R-0h</td><td>R-0h</td></tr></table>

Table 8-15. StatusRegister_2 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>6</td><td>OTP_ERR</td><td>R</td><td>0h</td><td>One Time Programmability Error
0h = No OTP error is detected
1h = OTP Error is detected</td></tr><tr><td>5</td><td>BUCK_OCP</td><td>R</td><td>0h</td><td>Buck Regulator Overcurrent Staus Bit
0h = No buck regulator overcurrent is detected
1h = Buck regulator overcurrent is detected</td></tr><tr><td>4</td><td>BUCK_UV</td><td>R</td><td>0h</td><td>Buck Regulator Undervoltage Staus Bit
0h = No buck regulator undervoltage is detected
1h = Buck regulator undervoltage is detected</td></tr><tr><td>3</td><td>VCP_UV</td><td>R</td><td>0h</td><td>Charge Pump Undervoltage Status Bit
0h = No charge pump undervoltage is detected
1h = Charge pump undervoltage is detected</td></tr><tr><td>2</td><td>SPI_PARITY</td><td>R-0</td><td>0h</td><td>SPI Parity Error Bit
0h = No SPI parity error is detected
1h = SPI parity error is detected</td></tr><tr><td>1</td><td>SPI_SCLK_FLT</td><td>R</td><td>0h</td><td>SPI Clock Framing Error Bit
0h = No SPI clock framing error is detected
1h = SPI clock framing error is detected</td></tr><tr><td>0</td><td>SPI_ADDR_FLT</td><td>R</td><td>0h</td><td>SPI Address Error Bit
0h = No SPI address fault is detected (due to accessing non-user register)
1h = SPI address fault is detected</td></tr></table>

## 8.6.2 CONTROL Registers

CONTROL Registers lists the memory-mapped registers for the CONTROL registers. All register offset addresses not listed in CONTROL Registers should be considered as reserved locations and the register contents should not be modified.

Table 8-16. CONTROL Registers  

<table><tr><td>Offset</td><td>Acronym</td><td>Register Name</td><td>Section</td></tr><tr><td>3h</td><td>ControlRegister_1</td><td>Control Register 1</td><td>Section 8.6.2.1</td></tr><tr><td>4h</td><td>ControlRegister_2A</td><td>Control Register 2A</td><td>Section 8.6.2.2</td></tr><tr><td>5h</td><td>ControlRegister_3</td><td>Control Register 3</td><td>Section 8.6.2.3</td></tr><tr><td>6h</td><td>ControlRegister_4</td><td>Control Register 4</td><td>Section 8.6.2.4</td></tr><tr><td>7h</td><td>ControlRegister_5</td><td>Control Register 5</td><td>Section 8.6.2.5</td></tr><tr><td>8h</td><td>ControlRegister_6</td><td>Control Register 6</td><td>Section 8.6.2.6</td></tr><tr><td>9h</td><td>ControlRegister_7</td><td>Control Register 7</td><td>Section 8.6.2.7</td></tr></table>

Table 8-16. CONTROL Registers (continued)  

<table><tr><td>Offset</td><td>Acronym</td><td>Register Name</td><td>Section</td></tr><tr><td>Ah</td><td>ControlRegister_8</td><td>Control Register 8</td><td>Section 8.6.2.8</td></tr><tr><td>Bh</td><td>ControlRegister_9</td><td>Control Register 9</td><td>Section 8.6.2.9</td></tr><tr><td>Ch</td><td>ControlRegister_10</td><td>Control Register 10</td><td>Section 8.6.2.10</td></tr></table>

Complex bit access types are encoded to fit into small table cells. CONTROL Access Type Codes show the codes that are used for access types in this section.

Table 8-17. CONTROL Access Type Codes  

<table><tr><td>Access Type</td><td>Code</td><td>Description</td></tr><tr><td colspan="3">Read Type</td></tr><tr><td>R</td><td>R</td><td>Read</td></tr><tr><td>R-0</td><td>R-0</td><td>ReadReturns 0s</td></tr><tr><td colspan="3">Write Type</td></tr><tr><td>W</td><td>W</td><td>Write</td></tr><tr><td>W1C</td><td>W1C</td><td>Write1 to clear</td></tr><tr><td>WAPU</td><td>WAPU</td><td>WriteAtomic write with password unlock</td></tr><tr><td colspan="3">Reset or Default Value</td></tr><tr><td>-n</td><td></td><td>Value after reset or the default value</td></tr></table>

### 8.6.2.1 ControlRegister_1 Register (Offset = 3h) [Reset = 00h]

ControlRegister1 is shown in ControlRegister1 Register and described in ControlRegister1 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-52. ControlRegister_1 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td></td><td colspan="4">RESERVED</td><td colspan="3">REG_LOCK</td></tr><tr><td></td><td colspan="4">R-0-0h</td><td colspan="3">R/WAPU-0h</td></tr></table>

Table 8-18. ControlRegister_1 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7-3</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>2-0</td><td>REG_LOCK</td><td>R/WAPU</td><td>0h</td><td>Register Lock Bits
0h = No effect unless locked or unlocked
1h = No effect unless locked or unlocked
2h = No effect unless locked or unlocked
3h = Write 011b to this register to unlock all registers
4h = No effect unless locked or unlocked
5h = No effect unless locked or unlocked
6h = Write 110b to lock the settings by ignoring further register writes except to these bits and address 0x03h bits 2-0.
7h = No effect unless locked or unlocked</td></tr></table>

### 8.6.2.2 ControlRegister_2A Register (Offset = 4h) [Reset = 80h]

ControlRegister2A is shown in ControlRegister_2A Register and described in ControlRegister_2A Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-53. ControlRegister_2A Register  

<table><tr><td>7 6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>RESERVED</td><td>SDO_MODE</td><td colspan="2">SLEW</td><td colspan="2">PWM_MODE</td><td>CLR_FLT</td></tr><tr><td>R/W-2h</td><td>R/W-0h</td><td colspan="2">R/W-0h</td><td colspan="2">R/W-0h</td><td>W1C-0h</td></tr></table>

Table 8-19. ControlRegister_2A Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7-6</td><td>RESERVED</td><td>R/W</td><td>2h</td><td>Reserved</td></tr><tr><td>5</td><td>SDO_MODE</td><td>R/W</td><td>0h</td><td>SDO Mode Setting
0h = SDO IO in Open Drain Mode
1h = SDO IO in Push Pull Mode</td></tr><tr><td>4-3</td><td>SLEW</td><td>R/W</td><td>0h</td><td>Slew Rate Settings
0h = Slew rate is 25 V/μs
1h = Slew rate is 50 V/μs
2h = Slew rate is 125 V/μs
3h = Slew rate is 200 V/μs</td></tr><tr><td>2-1</td><td>PWM_MODE</td><td>R/W</td><td>0h</td><td>Device Mode Selection
0h = Asynchronous rectification with analog Hall
1h = Asynchronous rectification with digital Hall
2h = Synchronous rectification with analog Hall
3h = Synchronous rectification with digital Hall</td></tr><tr><td>0</td><td>CLR_FLT</td><td>W1C</td><td>0h</td><td>Clear Fault
0h = No clear fault command is issued
1h = To clear the latched fault bits. This bit automatically resets after being written.</td></tr></table>

### 8.6.2.3 ControlRegister_3 Register (Offset = 5h) [Reset = 46h]

ControlRegister_3 is shown in ControlRegister_3 Register and described in ControlRegister_3 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-54. ControlRegister_3 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>RESERVED</td><td>RESERVED</td><td>RESERVED</td><td>PWM_100_DU TY_SEL</td><td>OVP_SEL</td><td>OVP_EN</td><td>RESERVED</td><td>OTW Replica</td></tr><tr><td>R-0-0h</td><td>R/W-1h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-1h</td><td>R/W-1h</td><td>R/W-0h</td></tr></table>

Table 8-20. ControlRegister_3 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>6</td><td>RESERVED</td><td>R/W</td><td>1h</td><td>Reserved</td></tr><tr><td>5</td><td>RESERVED</td><td>R/W</td><td>0h</td><td>Reserved</td></tr><tr><td>4</td><td>PWM_100_DUTY_SEL</td><td>R/W</td><td>0h</td><td>Frequency of PWM at 100% Duty Cycle
0h = 20KHz
1h = 40KHz</td></tr><tr><td>3</td><td>OVP_SEL</td><td>R/W</td><td>0h</td><td>Overvoltage Level Setting
0h = VM overvoltage level is 34-V
1h = VM overvoltage level is 22-V</td></tr><tr><td>2</td><td>OVP_EN</td><td>R/W</td><td>1h</td><td>Overvoltage Enable Bit
0h = Overvoltage protection is disabled
1h = Overvoltage protection is enabled</td></tr><tr><td>1</td><td>RESERVED</td><td>R/W</td><td>1h</td><td>Reserved</td></tr><tr><td>0</td><td>OTWRep</td><td>R/W</td><td>0h</td><td>Overtemperature Warning Reporting Bit
0h = Over temperature reporting on nFAULT is disabled
1h = Over temperature reporting on nFAULT is enabled</td></tr></table>

### 8.6.2.4 ControlRegister_4 Register (Offset = 6h) [Reset = 10h]

ControlRegister_4 is shown in ControlRegister_4 Register and described in ControlRegister_4 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-55. ControlRegister_4 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>DRV_OFF</td><td>OCP_CBC</td><td colspan="2">OCP_DEG</td><td>OCP_RETRY</td><td>OCP_LVL</td><td colspan="2">OCP_MODE</td></tr><tr><td>R/W-Oh</td><td>R/W-Oh</td><td colspan="2">R/W-1h</td><td>R/W-Oh</td><td>R/W-Oh</td><td colspan="2">R/W-Oh</td></tr></table>

Table 8-21. ControlRegister_4 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7</td><td>DRV_OFF</td><td>R/W</td><td>0h</td><td>Driver OFF Bit
0h = No Action
1h = Enter Low Power Standby Mode</td></tr><tr><td>6</td><td>OCP_CBC</td><td>R/W</td><td>0h</td><td>OCP PWM Cycle Operation Bit
0h = OCP clearing in PWM input cycle change is disabled
1h = OCP clearing in PWM input cycle change is enabled</td></tr><tr><td>5-4</td><td>OCP_DEG</td><td>R/W</td><td>1h</td><td>OCP Deglitch Time Settings
0h = OCP deglitch time is 0.2 μs
1h = OCP deglitch time is 0.6 μs
2h = OCP deglitch time is 1.25 μs
3h = OCP deglitch time is 1.6 μs</td></tr><tr><td>3</td><td>OCP_RETRY</td><td>R/W</td><td>0h</td><td>OCP Retry Time Settings
0h = OCP retry time is 5 ms
1h = OCP retry time is 500 ms</td></tr><tr><td>2</td><td>OCP_LVL</td><td>R/W</td><td>0h</td><td>Overcurrent Level Setting
0h = OCP level is 16 A
1h = OCP level is 24 A</td></tr><tr><td>1-0</td><td>OCP_MODE</td><td>R/W</td><td>0h</td><td>OCP Fault Options
0h = Overcurrent causes a latched fault
1h = Overcurrent causes an automatic retrying fault
2h = Overcurrent is report only but no action is taken
3h = Overcurrent is not reported and no action is taken</td></tr></table>

### 8.6.2.5 ControlRegister_5 Register (Offset = 7h) [Reset = 00h]

ControlRegister5 is shown in ControlRegister_5 Register and described in ControlRegister_5 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-56. ControlRegister_5 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>RESERVED</td><td>ILIM_RECIR</td><td>RESERVED</td><td>RESERVED</td><td>EN_AAR</td><td>EN_ASR</td><td>CSA_GAIN</td><td></td></tr><tr><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td></td></tr></table>

Table 8-22. ControlRegister_5 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7</td><td>RESERVED</td><td>R/W</td><td>0h</td><td>Reserved</td></tr><tr><td>6</td><td>ILIM_RECIR</td><td>R/W</td><td>0h</td><td>Current Limit Recirculation Settings
0h = Current recirculation through FETs (Brake Mode)
1h = Current recirculation through diodes (Coast Mode)</td></tr><tr><td>5</td><td>RESERVED</td><td>R/W</td><td>0h</td><td>Reserved</td></tr><tr><td>4</td><td>RESERVED</td><td>R/W</td><td>0h</td><td>Reserved</td></tr><tr><td>3</td><td>EN_AAR</td><td>R/W</td><td>0h</td><td>Active Asynchronous Rectification Enable Bit
0h = AAR mode is disabled
1h = AAR mode is enabled</td></tr><tr><td>2</td><td>EN_ASR</td><td>R/W</td><td>0h</td><td>Active Synchronous Rectification Enable Bit
0h = ASR mode is disabled
1h = ASR mode is enabled</td></tr><tr><td>1-0</td><td>CSA_GAIN</td><td>R/W</td><td>0h</td><td>Current Sense Amplifier&#x27;s Gain Settings
0h = CSA gain is 0.15 V/A
1h = CSA gain is 0.3 V/A
2h = CSA gain is 0.6 V/A
3h = CSA gain is 1.2 V/A</td></tr></table>

### 8.6.2.6 ControlRegister_6 Register (Offset = 8h) [Reset = 00h]

ControlRegister_6 is shown in ControlRegister_6 Register and described in ControlRegister_6 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-57. ControlRegister_6 Register  

<table><tr><td>7 6</td><td colspan="2">5 4</td><td colspan="2">3</td><td colspan="2">2 1</td><td>0</td></tr><tr><td>RESERVED</td><td>RESERVED</td><td>BUCK_PS_DIS</td><td colspan="2">BUCK_CL</td><td colspan="2">BUCK_SEL</td><td>BUCK_DIS</td></tr><tr><td>R-0-0h</td><td>R/W-0h</td><td>R/W-0h</td><td colspan="2">R/W-0h</td><td colspan="2">R/W-0h</td><td>R/W-0h</td></tr></table>

Table 8-23. ControlRegister_6 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7-6</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>5</td><td>RESERVED</td><td>R/W</td><td>0h</td><td>Reserved</td></tr><tr><td>4</td><td>BUCK_PS_DIS</td><td>R/W</td><td>0h</td><td>Buck Power Sequencing Disable Bit
0h = Buck power sequencing is enabled
1h = Buck power sequencing is disabled</td></tr><tr><td>3</td><td>BUCK_CL</td><td>R/W</td><td>0h</td><td>Buck Current Limit Setting
0h = Buck regulator current limit is set to 600 mA
1h = Buck regulator current limit is set to 150 mA</td></tr><tr><td>2-1</td><td>BUCK_SEL</td><td>R/W</td><td>0h</td><td>Buck Voltage Selection
0h = Buck voltage is 3.3 V
1h = Buck voltage is 5.0 V
2h = Buck voltage is 4.0 V
3h = Buck voltage is 5.7 V</td></tr><tr><td>0</td><td>BUCK_DIS</td><td>R/W</td><td>0h</td><td>Buck Disable Bit
0h = Buck regulator is enabled
1h = Buck regulator is disabled</td></tr></table>

### 8.6.2.7 ControlRegister_7 Register (Offset = 9h) [Reset = 00h]

ControlRegister_7 is shown in ControlRegister_7 Register and described in ControlRegister_7 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-58. ControlRegister_7 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td></td><td>RESERVED</td><td></td><td>HALL_HYS</td><td>BRAKE_MODE</td><td>COAST</td><td>BRAKE</td><td>DIR</td></tr><tr><td></td><td>R-0-0h</td><td></td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td><td>R/W-0h</td></tr></table>

Table 8-24. ControlRegister_7 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7-5</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>4</td><td>HALL_HYS</td><td>R/W</td><td>0h</td><td>Hall Comparator Hysteresis Settings
0h = 5 mV
1h = 50 mV</td></tr><tr><td>3</td><td>BRAKE_MODE</td><td>R/W</td><td>0h</td><td>Brake Mode Setting
0h = Device operation is braking in brake mode
1h = Device operation is coasting in brake mode</td></tr><tr><td>2</td><td>COAST</td><td>R/W</td><td>0h</td><td>Coast Bit
0h = Device coast mode is disabled
1h = Device coast mode is enabled</td></tr><tr><td>1</td><td>BRAKE</td><td>R/W</td><td>0h</td><td>Brake Bit
0h = Device brake mode is disabled
1h = Device brake mode is enabled</td></tr><tr><td>0</td><td>DIR</td><td>R/W</td><td>0h</td><td>Direction Bit
0h = Motor direction is set to clockwise direction
1h = Motor direction is set to anti-clockwise direction</td></tr></table>

### 8.6.2.8 ControlRegister_8 Register (Offset = Ah) [Reset = 00h]

ControlRegister_8 is shown in ControlRegister_8 Register and described in ControlRegister_8 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-59. ControlRegister_8 Register  

<table><tr><td>7 6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td>FGOUT_SEL</td><td>RESERVED</td><td>MTR_LOCK_R_ETRY</td><td colspan="2">MTR_LOCK_TDET</td><td colspan="2">MTR_LOCK_MODE</td></tr><tr><td>R/W-0h</td><td>R-0-0h</td><td>R/W-0h</td><td colspan="2">R/W-0h</td><td colspan="2">R/W-0h</td></tr></table>

Table 8-25. ControlRegister_8 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7-6</td><td>FGOUT_SEL</td><td>R/W</td><td>0h</td><td>Electrical Frequency Generation Output Mode Bits
0h = FGOUT frequency is 3x commutation frequency
1h = FGOUT frequency is 1x of commutation frequency
2h = FGOUT frequency is 0.5x of commutation frequency
3h = FGOUT frequency is 0.25x of commutation frequency</td></tr><tr><td>5</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>4</td><td>MTR_LOCK_RETRY</td><td>R/W</td><td>0h</td><td>Motor Lock Retry Settings
0h = 500 ms
1h = 5000 ms</td></tr><tr><td>3-2</td><td>MTR_LOCK_TDET</td><td>R/W</td><td>0h</td><td>Motor Lock Detection Time Settings
0h = 300 ms
1h = 500 ms
2h = 1000 ms
3h = 5000 ms</td></tr><tr><td>1-0</td><td>MTR_LOCK_MODE</td><td>R/W</td><td>0h</td><td>Motor Lock Fault Options
0h = Motor lock causes a latched fault
1h = Motor lock causes an automatic retrying fault
2h = Motor lock is report only but no action is taken
3h = Motor lock is not reported and no action is taken</td></tr></table>

### 8.6.2.9 ControlRegister_9 Register (Offset = Bh) [Reset = 00h]

ControlRegister_9 is shown in ControlRegister_9 Register and described in ControlRegister_9 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-60. ControlRegister_9 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td></td><td colspan="4">RESERVED</td><td colspan="3">ADVANCE_LVL</td></tr><tr><td></td><td colspan="4">R-0-0h</td><td colspan="3">R/W-0h</td></tr></table>

Table 8-26. ControlRegister_9 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7-3</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>2-0</td><td>ADVANCE_LVL</td><td>R/W</td><td>0h</td><td>Phase Advance Setting
0h = 0°
1h = 4°
2h = 7°
3h = 11°
4h = 15°
5h = 20°
6h = 25°
7h = 30°</td></tr></table>

### 8.6.2.10 ControlRegister_10 Register (Offset = Ch) [Reset = 00h]

ControlRegister_10 is shown in ControlRegister_10 Register and described in ControlRegister_10 Register Field Descriptions.

Return to the CONTROL Registers.

Figure 8-61. ControlRegister_10 Register  

<table><tr><td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td></tr><tr><td></td><td>RESERVED</td><td></td><td>DLYCMP_EN</td><td></td><td>DLY_TARGET</td><td></td><td></td></tr><tr><td></td><td>R-0-0h</td><td colspan="3">R/W-0h</td><td colspan="3">R/W-0h</td></tr></table>

Table 8-27. ControlRegister10 Register Field Descriptions  

<table><tr><td>Bit</td><td>Field</td><td>Type</td><td>Reset</td><td>Description</td></tr><tr><td>7-5</td><td>RESERVED</td><td>R-0</td><td>0h</td><td>Reserved</td></tr><tr><td>4</td><td>DLYCMP_EN</td><td>R/W</td><td>0h</td><td>Driver Delay Compensation enable
0h = Disable
1h = Enable</td></tr><tr><td>3-0</td><td>DLY_TARGET</td><td>R/W</td><td>0h</td><td>Delay Target for Driver Delay Compensation
0h = 0 us
1h = 0.4 us
2h = 0.6 us
3h = 0.8 us
4h = 1 us
5h = 1.2 us
6h = 1.4 us
7h = 1.6 us
8h = 1.8 us
9h = 2 us
Ah = 2.2 us
Bh = 2.4 us
Ch = 2.6 us
Dh = 2.8 us
Eh = 3 us
Fh = 3.2 us</td></tr></table>