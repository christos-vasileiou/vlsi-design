onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label clk -radix binary /TEST_FSMs/clk
add wave -noupdate -label reset -radix binary /TEST_FSMs/reset
add wave -noupdate -label start -radix binary /TEST_FSMs/start
add wave -noupdate -label i_process -radix binary /TEST_FSMs/i_process
add wave -noupdate -label i_byte_subs -radix binary /TEST_FSMs/i_byte_subs
add wave -noupdate -label i_shift_rows -radix binary /TEST_FSMs/i_shift_rows
add wave -noupdate -label i_mix_columns -radix binary /TEST_FSMs/i_mix_columns
add wave -noupdate -label i_key_addition -radix binary /TEST_FSMs/i_key_addition
add wave -noupdate -label i_initial_key_ready -radix binary /TEST_FSMs/i_initial_key_ready
add wave -noupdate -label i_round_key_ready -radix binary /TEST_FSMs/i_round_key_ready
add wave -noupdate -label i_data_received -radix binary /TEST_FSMs/i_data_received
add wave -noupdate -label i_finished -radix binary /TEST_FSMs/i_finished
add wave -noupdate -label i_done -radix binary /TEST_FSMs/i_done
add wave -noupdate -label main_fsm -radix hexadecimal /TEST_FSMs/main_fsm/main_state
add wave -noupdate -label o_load -radix binary /TEST_FSMs/o_load
add wave -noupdate -label o_process -radix binary /TEST_FSMs/o_process
add wave -noupdate -label o_send -radix binary /TEST_FSMs/o_send
add wave -noupdate -label o_finished -radix binary /TEST_FSMs/o_finished
add wave -noupdate -label o_add -radix binary /TEST_FSMs/o_add
add wave -noupdate -label o_substitute -radix binary /TEST_FSMs/o_substitute
add wave -noupdate -label o_shift_rows -radix binary /TEST_FSMs/o_shift_rows
add wave -noupdate -label o_columns -radix binary /TEST_FSMs/o_mix_columns
add wave -noupdate -label o_calc_init_round_key -radix binary /TEST_FSMs/o_calc_init_round_key
add wave -noupdate -label o_calc_round_key -radix binary /TEST_FSMs/o_calc_round_key
add wave -noupdate -label encr_fsm -radix hexadecimal /TEST_FSMs/encryption_fsm/encr_state
add wave -noupdate -label round_cnt -radix hexadecimal /TEST_FSMs/round_cnt
add wave -noupdate -label x /TEST_FSMs/x
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {1635 ns} 0}
configure wave -namecolwidth 172
configure wave -valuecolwidth 40
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
WaveRestoreZoom {1469 ns} {2453 ns}
