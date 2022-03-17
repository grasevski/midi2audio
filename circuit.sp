audio input and output

* op amp used for mapping voltage ranges and buffering
.include LM6132B.MOD

* diode used throughout
.model D1N4744 D

* negative rail created from arduino oscillator
.subckt negative osc gnd neg
C1 osc mid 10u
D1 mid gnd D1N4744
D2 neg mid D1N4744
C2 gnd neg 100u
.ends negative

* line level input, from a guitar for example
.subckt input snd pow neg gnd in
Xop opp opn pow neg opo LM6132B/NS
R1 input opn 100k
D1 opn opo D1N4744
D2 opo in D1N4744
R2 opn in 10k
R3 pow vd 200k
R4 vd opp 10k
R5 vd gnd 10k
.ends input

* line level output, to a guitar amp for example
.subckt output snd pow neg gnd out
Xop opp opn pow neg out LM6132B/NS
R1 snd opn 100k
R2 opn out 10k
Clpf opn out 10n
R3 pow opp 100k
R4 opp gnd 10k
.ends output

* arduino power source
Vin pow gnd DC 5

* oscillator from arduino D6 PWM pin
Vosc osc gnd PULSE(0 5 0 0 0 16u 32u)

* +-200mV input, from a guitar for example
Vsnd snd gnd SIN(0 200m 20)

* create negative rail from arduino
Xneg osc gnd neg negative

* arduino A0 pin
Rin in gnd 100k

* connect guitar to arduino
Xin snd pow neg gnd in input

* audio output from arduino D5 PWM pin
Vout out gnd SIN(2.5 2.5 20)

* connect arduino to guitar amplifier
Xout out pow neg gnd amp output

* guitar amplifier
Rout amp gnd 100k

.end
