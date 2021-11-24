onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label clk -radix binary /TEST_FSMs/fsms/clk
add wave -noupdate -label reset -radix binary /TEST_FSMs/fsms/reset
add wave -noupdate -label start -radix binary /TEST_FSMs/fsms/start
add wave -noupdate -divider -height 20 ENCR-FSM
add wave -noupdate -divider inputs
add wave -noupdate -label i_process -radix binary /TEST_FSMs/fsms/encryption_fsm/i_process
add wave -noupdate -label i_byte_subs -radix binary /TEST_FSMs/fsms/encryption_fsm/i_byte_subs
add wave -noupdate -label i_shift_rows -radix binary /TEST_FSMs/fsms/encryption_fsm/i_shift_rows
add wave -noupdate -label i_mix_columns -radix binary /TEST_FSMs/fsms/encryption_fsm/i_mix_columns
add wave -noupdate -label i_key_addition -radix binary /TEST_FSMs/fsms/encryption_fsm/i_key_addition
add wave -noupdate -label ecnr-state -radix hexadecimal /TEST_FSMs/fsms/encryption_fsm/encr_state
add wave -noupdate -divider outputs
add wave -noupdate -label round_cnt -radix hexadecimal /TEST_FSMs/fsms/encryption_fsm/round_cnt
add wave -noupdate -label o_finished -radix binary /TEST_FSMs/fsms/encryption_fsm/o_finished
add wave -noupdate -label o_substitute -radix binary /TEST_FSMs/fsms/encryption_fsm/o_substitute
add wave -noupdate -label o_shift_rows -radix binary /TEST_FSMs/fsms/encryption_fsm/o_shift_rows
add wave -noupdate -label o_mix_columns -radix binary /TEST_FSMs/fsms/encryption_fsm/o_mix_columns
add wave -noupdate -label o_add -radix binary /TEST_FSMs/fsms/encryption_fsm/o_add
add wave -noupdate -label o_calc_round_key -radix binary /TEST_FSMs/fsms/encryption_fsm/o_calc_round_key
add wave -noupdate -divider -height 20 MAIN-FSM
add wave -noupdate -divider inputs
add wave -noupdate -label i_finished -radix binary /TEST_FSMs/fsms/main_fsm/i_finished
add wave -noupdate -label i_done -radix binary /TEST_FSMs/fsms/main_fsm/i_done
add wave -noupdate -label i_data_received_text -radix binary /TEST_FSMs/fsms/main_fsm/i_data_received_text
add wave -noupdate -label i_data_received_key -radix binary /TEST_FSMs/fsms/main_fsm/i_data_received_key
add wave -noupdate -label main-state -radix hexadecimal /TEST_FSMs/fsms/main_fsm/main_state
add wave -noupdate -divider outputs
add wave -noupdate -label o_load -radix binary /TEST_FSMs/fsms/main_fsm/o_load
add wave -noupdate -label o_process -radix binary /TEST_FSMs/fsms/main_fsm/o_process
add wave -noupdate -label o_send -radix binary /TEST_FSMs/fsms/main_fsm/o_send
add wave -noupdate -divider iterator
add wave -noupdate -label x /TEST_FSMs/x
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {490 ns} 0}
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ns} {1100 ns}
