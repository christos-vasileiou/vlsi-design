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

read_file -format ddc {/home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/Design/mips-SCANNED.ddc}

write_test_protocol -output netlist/mips-SCANNED.spf
write_test_protocol -output netlist/mips-SCANNED.stil
write_sdf netlist/mips-SCANNED.sdf
write -format verilog -h -o netlist/mips-SCANNED.v
write_sdc netlist/mips-SCANNED.sdc

report_power > reports/report_power-SCANNED.txt
report_timing -transition_time -delay_type max -significant_digits 10 > reports/report_timing-SCANNED.txt
report_area -designware -physical > reports/report_area-SCANNED.txt
report_design -physical > reports/report_design-SCANNED.txt
