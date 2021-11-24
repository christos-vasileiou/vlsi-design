
#all verilog files
set my_verilog_files { /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/adder_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/alu_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/alu_control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/data_mem_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/flush_control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/forw_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/instr_mem_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/jr_control_unit.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/mux2x5to5_b.v  /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/mux3x32to32_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/myDivider_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/register_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/sign_extend_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/stage.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/stall_control_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/wb_forward_unit_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/zero_extension_b.v /home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/src/mips32_b/top_b.v }

#Top level module
set my_toplevel MIPSpipeline
#/* Motorola CDR 3 library: */

define_design_lib WORK -path ./WORK

set search_path [ list "/home/faculty/chsotiriou/IHP-Lib/IHP-0.25um/digital/synopsys/" ]
######################################   MAX library - Setup Violation ###########################################

echo "################## Start synthesis for SETUP-time violation ##################\n\n\n"
set target_library sgc25a.max.db 
set symbol_library sgc25a.sdb
set link_library [concat  "*" $target_library]

# /* add I/O library: */
set target_library [concat  $target_library sgc25a_io.max.db ]
set symbol_library [format "%s%s"  $symbol_library sgc25a_io.sdb]
set link_library [concat  "*" $target_library]

analyze -library WORK -format verilog $my_verilog_files
elaborate $my_toplevel -architecture verilog -library WORK

set_drive [drive_of cdr3synPwcslV225T125/inv_1/x] [all_inputs]
set_load [load_of cdr3synPwcslV225T125/inv_1/a] [all_outputs]
create_clock "clk" -name clk -period 20
create_clock "div_clock" -name div_clock -period 10
set_false_path -from reset
compile -map_effort high

######################################   Set dft - scan dFF ###########################################
create_clock "clk" -name clk -period 64
create_clock "div_clock" -name div_clock -period 32
compile -map_effort high -incremental_mapping -scan

set_scan_configuration 
create_test_protocol -infer_asynch -infer_clock
set_dft_configuration -fix_reset enable -fix_clock enable
insert_dft
dft_drc -coverage_estimate 

#write -format verilog -h -o netlist/mips_without_scans.v
#write_sdc netlist/mips_without_scans.sdc

write_test_protocol -output netlist/mips-SCANNED.spf
write_test_protocol -output netlist/mips-SCANNED.stil
write_sdf netlist/mips_without_scans.sdf
write -format verilog -h -o netlist/mips_without_scans.v
write_sdc netlist/mips_without_scans.sdc

report_power > reports/report_power-SCANNED.txt
report_timing -transition_time -delay_type max -significant_digits 10 > reports/report_timing_without_scans.txt
report_area -designware -physical > reports/report_area_without_scans.txt
report_design -physical > reports/report_design_without_scans.txt
