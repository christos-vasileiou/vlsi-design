/*
 *  --------------------------------
 * | Name: Christos Vasileiou       |
 * | NetID: cxv200006               |
 * | e-mail: cxv200006@utdallas.edu |
 *  --------------------------------
 *
 * Description:
 * The implementation of 2 FSMs.
 * 
 */
 
module FSMs(
    input clk, reset, start,
    input i_done, i_data_received_key, 
    input i_data_received_text,
    input [127:0] data,
    input i_byte_subs, i_shift_rows,
    input i_mix_columns,
    input i_key_addition, i_round_key_get_ready,
    output [3:0] round_cnt,
    output o_add, o_substitute, o_shift_rows, o_mix_columns,
    output o_calc_round_key, o_send, o_load,
    output reg [127:0] cipher_text
  );
  
  wire o_process, o_finished;
  reg i_process, i_finished;
  reg [127:0] text, key;

  MAIN_FSM main_fsm (
    .clk(clk), .reset(reset), .start(start),
    .i_data_received_text(i_data_received_text), 
    .i_data_received_key(i_data_received_key), 
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
    .i_key_addition(i_key_addition),
    .i_round_key_get_ready(i_round_key_get_ready),
    .round_cnt(round_cnt), 
    .o_substitute(o_substitute),
    .o_shift_rows(o_shift_rows),
    .o_mix_columns(o_mix_columns),
    .o_add(o_add),
    .o_finished(o_finished),
    .o_calc_round_key(o_calc_round_key)
  );

  always@(posedge clk)
  begin
    if (reset)
    begin
      text <= 0;
      key <= 0;
      i_process <= 0;
      i_finished <= 0;
      cipher_text <= 0;
    end
    else
    begin
      i_process <= o_process;
      i_finished <= o_finished;
      cipher_text <= (o_send) ? text ^ key : 0;
      if (i_data_received_text)
      begin
        text <= data;
      end
      else
      begin
        if (i_data_received_text)
        begin
          key <= data;
        end
        else
        begin
          text <= text;
          key <= key;
        end
      end
    end
  end
     
endmodule

// FSM stages encoding: Gray-Code
`define IDLE 3'b000
`define RECEIVE_TEXT 3'b001
`define RECEIVE_KEY 3'b011
`define PROCESS 3'b010
`define SEND 3'b110
`define DONE 3'b100

module MAIN_FSM(
    input clk, reset, start,
    input i_data_received_text, 
    input i_data_received_key, 
    input i_finished, 
    input i_done, 
    output reg o_load, 
    output reg o_process, 
    output reg o_send
  );

  reg [2:0] main_state;

  always @(posedge clk)
  begin
    if (reset)
    begin
      main_state <= `IDLE;
      o_load <= 0;
      o_process <= 0;
      o_send <= 0;
    end
    else 
    begin
      case (main_state)
        `IDLE: begin
          if (start)
          begin
          // Give me the text! RECEIVE_TEXT
            main_state <= `RECEIVE_TEXT;
            o_load <= 1;
          end
        end
        `RECEIVE_TEXT: begin
          if (i_data_received_text)
          begin
          // Give me the key! RECEIVE_KEY
            main_state <= `RECEIVE_KEY;
            o_load <= 0;
          end
        end
        `RECEIVE_KEY: begin
          if (i_data_received_key)
          begin
          // I am starting the process and the AES rounds! PROCESS
            main_state <= `PROCESS;
            o_process <= 1;
            o_load <= 0;
          end
        end
        `PROCESS: begin
          if (i_finished)
          begin
          // I am done. Cipher data is ready to be sent! SEND
            main_state <= `SEND;
            o_process <= 0;
            o_send <= 1;
          end
        end
        `SEND: begin
          if (i_done)
          begin  
            main_state <= `IDLE;
            o_send <= 0;
          end
        end  
        default: begin
          main_state <= main_state;
          o_process <= 0;
          o_load <= 0;
          o_send <= 0;
        end
      endcase    
    end
  end  
  
endmodule


// encryption FSM stages encoding: Gray-Code
`define WAIT 3'b000
`define KEY_ADDITION 3'b001
`define ROUND_KEY 3'b011
`define BYTE_SUBS 3'b010
`define SHIFT_ROWS 3'b110
`define MIX_COLUMNS 3'b100

module ENCR_FSM(
    input clk, reset,
    input i_process,
    input i_byte_subs,
    input i_shift_rows,
    input i_mix_columns,
    input i_key_addition,
    input i_round_key_get_ready,
    output reg [3:0] round_cnt,   
    output reg o_finished, 
    output reg o_add,
    output reg o_substitute,
    output reg o_shift_rows,
    output reg o_mix_columns,
    output reg o_calc_round_key
  );

  /*  key lengths  |  # rounds
   * --------------------------
   *   128-bits    |     10+1    
   *   192-bits    |     12+1    
   *   256-bits    |     14+1    
   *  
   *  (+1) The stage of first round consists of KeyAddition Layer, ONLY. 
   *  So, round_cnt 4-bits -> 2^4
   */

  reg [2:0] encr_state;
  wire key_ready = (i_round_key_get_ready == 1'b1);

  always@(posedge clk)
  begin
    if (reset)
    begin
      encr_state <= `WAIT;
      round_cnt <= -1;
      o_finished <= 1'b0;
      o_substitute <= 1'b0;
      o_add <= 1'b0;
      o_shift_rows <= 1'b0;
      o_mix_columns <= 1'b0;
      o_calc_round_key <= 1'b0;
    end
    else 
    begin
      case (encr_state)
        `WAIT: begin
          if (i_process == 1'b1 & i_key_addition == 1'b1) 
          begin
            encr_state <= `KEY_ADDITION; // Initial Round
            //round_cnt <= round_cnt + 1;
            o_add <= 1'b1;
            o_finished <= 1'b0;
          end
        end
        `KEY_ADDITION: begin
          if (key_ready == 1'b1 & round_cnt == 4'b1001)
          begin
            o_finished <= 1'b1;
            round_cnt <= round_cnt + 1;
            encr_state <= `WAIT;
            o_calc_round_key <= 1'b0;
            o_add <= 1'b0;
          end
          else if (key_ready == 1'b1 & round_cnt != 4'b1001)
          begin
            o_calc_round_key <= 1'b1;
            round_cnt <= round_cnt + 1;
            encr_state <= `ROUND_KEY;
            o_add <= 1'b0;
          end
        end
        `ROUND_KEY: begin
          if (i_byte_subs)
          begin
            //o_calc_init_round_key <= 1'b0;
            o_calc_round_key <= 1'b0;
            encr_state <= `BYTE_SUBS;
            o_add <= 1'b0;
            o_substitute <= 1'b1;
          end
        end
        `BYTE_SUBS: begin
          if (i_shift_rows)
          begin
            encr_state <= `SHIFT_ROWS;
            o_substitute <= 1'b0;
            o_shift_rows <= 1'b1;
          end
        end
        `SHIFT_ROWS: begin
          if (round_cnt != 4'b1001 & i_mix_columns)
          begin
            encr_state <= `MIX_COLUMNS;
            o_shift_rows <= 1'b0;
            o_mix_columns <= 1'b1;
          end
          else if (round_cnt == 4'b1001 & i_key_addition == 1'b1)
          begin
            encr_state <= `KEY_ADDITION;
            //round_cnt <= round_cnt + 1;
            o_shift_rows <= 1'b0;
            o_mix_columns <= 1'b0;
            o_add <= 1'b1;
          end
        end
        `MIX_COLUMNS: begin
          if (i_key_addition == 1'b1)
          begin
            //o_calc_round_key <= 1'b1;
            encr_state <= `KEY_ADDITION;
            //round_cnt <= round_cnt + 1;
            o_mix_columns <= 1'b0;
            o_add <= 1'b1;
          end 
        end
        default: begin
          encr_state <= `WAIT;
          round_cnt <= -1;
          o_finished <= 1'b0;
          o_substitute <= 1'b0;
          o_add <= 1'b0;
          o_shift_rows <= 1'b0;
          o_mix_columns <= 1'b0;
          o_calc_round_key <= 1'b0;
          //o_calc_init_round_key <= 1'b0;
        end
      endcase
    end
  end
endmodule
