* File: /home/eng/c/cxv200006/cad/gf65/inv_LVS/inv.pex.sp
* Created: Mon Mar  8 23:46:12 2021
* Program "Calibre xRC"
* Version "v2013.2_18.13"
* 
.include "/home/eng/c/cxv200006/cad/gf65/inv_LVS/inv.pex.sp.pex"
.subckt inv  GND! OUT VDD! IN
* 
* IN	IN
* VDD!	VDD!
* OUT	OUT
* GND!	GND!
XD0_noxref N_GND!_D0_noxref_pos N_VDD!_D0_noxref_neg DIODENWX  AREA=1.20912e-12
+ PERIM=4.472e-06
XMMN0 N_OUT_MMN0_d N_IN_MMN0_g N_GND!_MMN0_s N_GND!_D0_noxref_pos NFET L=6e-08
+ W=1.2e-07 AD=3e-14 AS=2.52e-14 PD=7.4e-07 PS=6.6e-07 NRD=1.45833 NRS=1.125 M=1
+ NF=1 CNR_SWITCH=0 PCCRIT=0 PAR=1 PTWELL=0 SA=2.1e-07 SB=2.5e-07 SD=0 PANW1=0
+ PANW2=0 PANW3=0 PANW4=0 PANW5=0 PANW6=0 PANW7=7.2e-15 PANW8=0 PANW9=0 PANW10=0
XMMP0 N_OUT_MMP0_d N_IN_MMP0_g N_VDD!_MMP0_s N_VDD!_D0_noxref_neg PFET L=6e-08
+ W=3.6e-07 AD=9e-14 AS=7.56e-14 PD=1.22e-06 PS=1.14e-06 NRD=0.486111
+ NRS=0.277778 M=1 NF=1 CNR_SWITCH=0 PCCRIT=0 PAR=1 PTWELL=1 SA=2.1e-07
+ SB=2.5e-07 SD=0 PANW1=2.1e-15 PANW2=3e-15 PANW3=8.04e-15 PANW4=1.956e-14
+ PANW5=1.38e-14 PANW6=1.68e-14 PANW7=7.8e-15 PANW8=1.2e-14 PANW9=3.3e-15
+ PANW10=0
*
.include "/home/eng/c/cxv200006/cad/gf65/inv_LVS/inv.pex.sp.INV.pxi"
*
.ends
*
*
