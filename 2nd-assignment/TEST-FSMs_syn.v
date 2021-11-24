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
 
module TEST_FSMs_syn;
  
  reg [127:0] data;
  reg clk, reset, start, i_process, i_byte_subs, i_shift_rows, i_mix_columns;
  reg i_key_addition, i_round_key_get_ready;
  reg i_data_received_key, i_data_received_text, i_finished, i_done;
  wire [2:0] main_state;
  wire o_load, o_process, o_send, o_finished, o_add, o_substitute, o_shift_rows; 
  wire o_mix_columns, o_calc_init_round_key, o_calc_round_key;
  wire [2:0] encr_state;
  wire [3:0] round_cnt;
  wire [127:0] cipher_text, prime_key;
  integer x;
   
  FSMs_1 fsms(
    .clk(clk), 
    .reset(reset), 
    .start(start),
    .i_done(i_done), 
    .i_data_received_text(i_data_received_text),
    .i_data_received_key(i_data_received_key),
    .data(data),
    .i_byte_subs(i_byte_subs), 
    .i_shift_rows(i_shift_rows), 
    .i_mix_columns(i_mix_columns), 
    .i_key_addition(i_key_addition),
    .i_round_key_get_ready(i_round_key_get_ready),
    .round_cnt(round_cnt),
    .o_add(o_add), 
    .o_substitute(o_substitute), 
    .o_shift_rows(o_shift_rows), 
    .o_mix_columns(o_mix_columns), 
    .o_calc_round_key(o_calc_round_key), 
    .o_send(o_send), 
    .o_load(o_load),
    .cipher_text(cipher_text), 
    .prime_key(prime_key)
  );
   
  initial begin
    clk=0;
    reset=1; //initialize the registers of the FSMs
    start=0; 
    i_data_received_key=0;
    i_data_received_text=0;
    i_finished=0;
    i_done=0;
    i_process=0;
    i_byte_subs=0;
    i_shift_rows=0;
    i_mix_columns=0;
    i_key_addition=0;
    i_round_key_get_ready=0;
    
    #100;
    reset = 0; // At this time reset is free.
    #(`PERIOD);
    start=1; //The 1st FSM (MAIN_FSM) is starting now. It rises the o_load to Load the plain text and key.
    #(`PERIOD);
    start=0;
    data = 128'h5477_6F20_4F6E_6520_4E69_6E65_2054_776F;
    i_data_received_text = 1; // Data received. 
    #(`PERIOD);
    start=0;
    i_data_received_text = 0;
    data = 128'h5468_6174_7320_6D79_204B_756E_6720_4675;
    i_data_received_key = 1; // Data received. 
    #(`PERIOD);
    i_data_received_key = 0; 
    #(`PERIOD);
    i_key_addition = 1;
    #(`PERIOD);
    i_key_addition = 0;
    #(`PERIOD);
    i_round_key_get_ready = 1; 
    #(`PERIOD);
    i_round_key_get_ready = 0;
    #(`PERIOD*3);
    
    for (x=0; x<11; x=x+1)
    begin
      #(`PERIOD);
      i_byte_subs = 1; // byte substituion
      #(`PERIOD);
      i_byte_subs = 0;
      #(`PERIOD);
      i_shift_rows = 1; // shifting rows
      #(`PERIOD);
      i_shift_rows = 0;
      #(`PERIOD);
      i_mix_columns = 1; // mix columns
      #(`PERIOD);
      i_mix_columns = 0;
      #(`PERIOD);
      i_key_addition = 1;
      #(`PERIOD);
      i_key_addition = 0;
      #(`PERIOD);
      i_round_key_get_ready = 1; // addition
      #(`PERIOD);
      i_round_key_get_ready = 0;
      #(`PERIOD*3);
    end // end for loop
    #(`PERIOD*4);
    i_done = 1; // Cipher-Code is sent! It's done!
    #(`PERIOD*2);
    $finish;
    
  end 
  
  always #(`PERIOD/2) clk = ~clk;
  
endmodule