audio input and output

* STM32 power source
Vdd pow gnd DC 3.3

* +-200mV input, from a guitar for example
Vsnd snd gnd SIN(0 200m 20)

* STM32 opamp pin
Ra0 in gnd 1M

* connect guitar to STM32
Cin snd in 100n

* audio output from STM32 dac pin
Va3 out gnd SIN(1 1 20)

* connect STM32 to guitar amplifier
Cout out amp 100n

* remove DC offset
Rhpf amp gnd 100k

* guitar amplifier
Ra4 amp gnd 1M

.end
