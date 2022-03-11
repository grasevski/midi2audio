audio input and output

* op amp used to change the signal voltage ranges
.include LM6132B.MOD

* diode used to clamp the input to positive voltage
.model D1N4744 D

* arduino power source
Vin pow gnd DC 5

* line level input, from a guitar for example
.subckt input pow gnd
Vsnd snd gnd SIN(-1 2 20)
Xop opp opn pow gnd opo LM6132B/NS
C1 snd opp 10u
D1 opp gnd D1N4744
R1 gnd opn 10k
R2 opn opo 18k
Rout opo gnd 100k
.ends input

* line level output, to a guitar amp for example
.subckt output pow gnd
Vdac dac gnd SIN(0 2.5 20)
Vref ref gnd DC 1.25
C1 dac hpf 10u
R1 hpf div 30k
R2 div gnd 120k
Rout hpf gnd 100k
.ends output

* connect audio input circuit to arduino power supply
Xin pow gnd input

* connect audio output circuit to arduino power supply
Xout pow gnd output

.end
