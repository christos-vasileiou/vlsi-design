/*
 *  --------------------------------
 * | Name: Christos Vasileiou       |
 * | NetID: cxv200006               |
 * | e-mail: cxv200006@utdallas.edu |
 *  --------------------------------
 *
 * Description:
 * Testbench for the AES encryption algorithm implementation wth 2 FSMs.
 * 
 */
 
 `define PERIOD 20
 
 module TEST_FSMs;
   
   reg clk, reset, start, i_process, i_byte_subs, i_shift_rows, i_mix_columns, i_key_addition, i_initial_key_ready, i_round_key_ready;
   reg i_data_received, i_finished, i_done;
   wire [2:0] main_state;
   wire o_load, o_process, o_send, o_finished, o_add, o_substitute, o_shift_rows, o_mix_columns, o_calc_init_round_key, o_calc_round_key;
   wire [2:0] encr_state;
   wire [3:0] round_cnt;
   
   integer x;
   
   
   MAIN_FSM main_fsm (
     .clk(clk), .reset(reset), .start(start),
     .i_data_received(i_data_received), 
     .i_finished(i_finished), 
     .i_done(i_done),
     .o_load(o_load), 
     .o_process(o_process), 
     .o_send(o_send)
   );
   
   ENCR_FSM encryption_fsm (
     .clk(clk), .reset(reset),
     .i_process(i_process),
     .i_byte_subs(i_byte_subs),
     .i_shift_rows(i_shift_rows),
     .i_mix_columns(i_mix_columns),
     .i_initial_key_ready(i_initial_key_ready),
     .i_key_addition(i_key_addition),
     .i_round_key_ready(i_round_key_ready),
     .round_cnt(round_cnt), 
     .o_substitute(o_substitute),
     .o_shift_rows(o_shift_rows),
     .o_mix_columns(o_mix_columns),
     .o_add(o_add),
     .o_finished(o_finished),
     .o_calc_init_round_key(o_calc_init_round_key),
     .o_calc_round_key(o_calc_round_key)
   );
  
  initial begin
    clk=0;
    reset=1; //initialize the registers of the FSMs
    start=0; 
    i_data_received=0;
    i_finished=0;
    i_done=0;
    i_process=0;
    i_byte_subs=0;
    i_shift_rows=0;
    i_mix_columns=0;
    i_key_addition=0;
    i_initial_key_ready=0;
    i_round_key_ready=0;
    
    #100
    reset = 0; // At this time reset is free.
    #(`PERIOD)
    start=1; //The 1st FSM (MAIN_FSM) is starting now. It rises the o_load to Load the plain text and key.
    #(`PERIOD)
    start=0;
    i_data_received = 1; // Data received. Enable the 2nd FSM (ENCR_FSM)
    #(`PERIOD*2)
    i_data_received = 0; 
    i_process = o_process; // connect the active signal to each other. Enable the calculation of the 1st round key and the addition (XOR)
    #(`PERIOD)
    i_key_addition = 1;
    #(`PERIOD)
    i_key_addition = 0;
    i_byte_subs = 1; // Enable byte substitution which takes the bytes from SBOX.
    #(`PERIOD)
    i_byte_subs = 0;
    i_shift_rows = 1; // Enable the byte shifting for each 32-bits word.
    #(`PERIOD)
    i_shift_rows = 0;
    i_mix_columns = 1; // Enable the GF(2^8) multiplication of the array with each 32-bits.
    #(`PERIOD)
    i_mix_columns = 0;
    // waits until the 1st round_key and mix columns get ready
    #(`PERIOD)
    i_initial_key_ready = 1; 
    #(`PERIOD)
    i_key_addition = 1;
    
    for (x=0; x<11; x=x+1)
    begin
      #(`PERIOD)
      i_initial_key_ready = 0;
      i_round_key_ready=0;
      i_key_addition = 0;
      i_byte_subs = 1; // byte substituion
      #(`PERIOD)
      i_byte_subs = 0;
      i_shift_rows = 1; // shifting rows
      #(`PERIOD)
      i_shift_rows = 0;
      i_mix_columns = 1; // mix columns
      #(`PERIOD)
      i_mix_columns = 0;
      #(`PERIOD)
      i_round_key_ready = 1; // addition
      #(`PERIOD)
      i_key_addition = 1;
    end // end for loop
    i_finished = o_finished; // The process of the encyprion has finished. Send the cipher-code
    i_process = o_process; 
    #(`PERIOD*4)
    i_done = 1; // Cipher-Code is sent! It's done!
    #(`PERIOD*2)
    $finish;
    
  end 
  
  always #(`PERIOD/2) clk = ~clk;
  
endmodule