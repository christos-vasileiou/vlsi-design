
#all verilog files
set my_verilog_files { /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/adder_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/alu_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/alu_control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/data_mem_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/flush_control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/forw_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/instr_mem_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/jr_control_unit.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/mux2x5to5_b.v  /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/mux3x32to32_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/myDivider_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/register_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/sign_extend_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/stage.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/stall_control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/wb_forward_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/zero_extension_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/top_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/mux2x32to32_b.v}

#Top level module
set my_toplevel MIPSpipeline

#target frequency in MHz for optimizationS
#set my_clk_freq_MHz 250

#Delay of input signals (clk-to-Q, Package etc.)
#set my_input_delay_ns 0.1

#reserved time for output signals
#set my_output_delay_ns 0.1

#/* Motorola CDR 3 library: */

define_design_lib WORK -path ./WORK

set target_library /home/faculty/chsotiriou/IHP-Lib/IHP-0.25um/digital/synopsys/sgc25a.min.db 
set symbol_library /home/faculty/chsotiriou/IHP-Lib/IHP-Kit/db/cdr3.sdb
set link_library /home/faculty/chsotiriou/IHP-Lib/IHP-0.25um/digital/synopsys/sgc25a.min.db 

#set target_library [concat $target_library {/home/faculty/chsotiriou/IHP-Lib/IHP-Kit/db/cdr3_io.min.db /home/faculty/chsotiriou/IHP-Lib/IHP-Kit/SRAM/sram16k_pin.max.db} ]
#set symbol_library [format "%s %s" $symbol_library /home/faculty/chsotiriou/IHP-Lib/IHP-Kit/db/cdr3_io.sdb]
#set link_library [concat  "*" $target_library]

#set period in ps
#set my_period_ps [expr 1000000 / $my_clk_freq_MHz]

analyze -library WORK -format verilog $my_verilog_files

elaborate $my_toplevel -architecture verilog -library WORK

set_drive [drive_of cdr3synPwcslV225T125/inv_1/x] [all_inputs]
set_load [load_of cdr3synPwcslV225T125/inv_1/a] [all_inputs]

create_clock "clk" -name clk -period 21
create_clock "div_clock" -name div_clock -period 10.5

uniquify
link

compile -area_effort high -map_effort high 

write_file -format verilog -hierarchy -output topmips_b_bc.v
write_sdf topmips_b_bc.sdf
write_saif -rtl -output topmips_b_bc.saif
