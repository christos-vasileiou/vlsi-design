onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label clk /TEST_FSMs_syn/fsms/clk
add wave -noupdate -label reset /TEST_FSMs_syn/fsms/reset
add wave -noupdate -label start /TEST_FSMs_syn/fsms/start
add wave -noupdate -divider -height 20 ENCR-FSM
add wave -noupdate -divider inputs
add wave -noupdate -label i_byte_subs /TEST_FSMs_syn/fsms/i_byte_subs
add wave -noupdate -label i_shift_rows /TEST_FSMs_syn/fsms/i_shift_rows
add wave -noupdate -label i_mix_columns /TEST_FSMs_syn/fsms/i_mix_columns
add wave -noupdate -label i_key_addition /TEST_FSMs_syn/fsms/i_key_addition
add wave -noupdate -divider outputs
add wave -noupdate -label ecnr_state -radix hexadecimal {/TEST_FSMs_syn/fsms/\encryption_fsm/encr_state }
add wave -noupdate -label o_substitute /TEST_FSMs_syn/fsms/o_substitute
add wave -noupdate -label o_shift_rows /TEST_FSMs_syn/fsms/o_shift_rows
add wave -noupdate -label o_mix_columns /TEST_FSMs_syn/fsms/o_mix_columns
add wave -noupdate -label o_add /TEST_FSMs_syn/fsms/o_add
add wave -noupdate -label o_calc_round_key /TEST_FSMs_syn/fsms/o_calc_round_key
add wave -noupdate -label o_finished /TEST_FSMs_syn/fsms/o_finished
add wave -noupdate -expand -group round_counter -label {round_cnt[3]} {/TEST_FSMs_syn/fsms/\encryption_fsm/round_cnt_reg[3] /q}
add wave -noupdate -expand -group round_counter -label {round_cnt[2]} {/TEST_FSMs_syn/fsms/\encryption_fsm/round_cnt_reg[2] /q}
add wave -noupdate -expand -group round_counter -label {round_cnt[1]} {/TEST_FSMs_syn/fsms/\encryption_fsm/round_cnt_reg[1] /q}
add wave -noupdate -expand -group round_counter -label {round_cnt[0]} {/TEST_FSMs_syn/fsms/\encryption_fsm/round_cnt_reg[0] /q}
add wave -noupdate -divider -height 20 MAIN-FSM
add wave -noupdate -divider inputs
add wave -noupdate -label i_done /TEST_FSMs_syn/fsms/i_done
add wave -noupdate -label i_data_received_key /TEST_FSMs_syn/fsms/i_data_received_key
add wave -noupdate -label i_data_received_text /TEST_FSMs_syn/fsms/i_data_received_text
add wave -noupdate -label i_finished /TEST_FSMs_syn/fsms/i_finished
add wave -noupdate -divider outputs
add wave -noupdate -label main_state -radix hexadecimal {/TEST_FSMs_syn/fsms/\main_fsm/main_state }
add wave -noupdate -label o_send /TEST_FSMs_syn/fsms/o_send
add wave -noupdate -label o_load /TEST_FSMs_syn/fsms/o_load
add wave -noupdate -label o_process /TEST_FSMs_syn/fsms/o_process
add wave -noupdate -divider iterator
add wave -noupdate -label x /TEST_FSMs_syn/x
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {368 ns} 0}
configure wave -namecolwidth 144
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
WaveRestoreZoom {0 ns} {3486 ns}
