audio input and output
.include LM6132B.MOD
Vin pow gnd DC 5

.subckt input pow gnd
Vsnd snd gnd SIN(-1 2 20)
Xop opp opn pow gnd opo LM6132B/NS
R1 snd opn 12k
R2 opn opo 30k
R3 pow opp 120k
R4 opp gnd 20k
Rout opo gnd 100k
.ends input

.subckt output pow gnd
Vdac dac gnd SIN(0 2.5 20)
Vref ref gnd DC 1.25
Xop opp opn pow gnd opo LM6132B/NS
R1 dac opn 15k
R2 opn opo 12k
R3 ref opp 15k
R4 opp gnd 12k
Rout opo gnd 100k
.ends output

Xin pow gnd input
Xout pow gnd output
.end
