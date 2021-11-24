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

read_file -format ddc {/home/inf2014/chrivasileiou/ce333/SYNTHESIS_SYNOPSYS_DC/Design/mips_without_scan.ddc}

set_drive [drive_of cdr3synPwcslV225T125/inv_1/x] [all_inputs]
set_load [load_of cdr3synPwcslV225T125/inv_1/a] [all_outputs]
create_clock "clk" -name clk -period 64
create_clock "div_clock" -name div_clock -period 32
set_false_path -from reset
compile -map_effort high -incremental_mapping -scan

set_scan_configuration 
create_test_protocol -infer_asynch -infer_clock
set_dft_configuration -fix_reset enable -fix_clock enable
insert_dft
dft_drc -coverage_estimate 

start_gui
