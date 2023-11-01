`timescale 1ns/1ns

module testbench();

logic clk, reset, en;

always begin
	#10
	clk = !clk;
end

always begin
	#300
	en = ~en;
	#10
	en = ~en;
end

logic [31:0] a_1, b_1, c_1, q_1, a_2, b_2, c_2, q_2;

// control unit data ports
logic [31:0] x_in_b, y_in_b, vx_in_b, vy_in_b;
logic [31:0] x, y, vx, vy, px, py;

// memory unit data ports
logic [27:0] x_in; 
logic [26:0] y_in; 
logic [20:0] vx_in, vy_in;

logic [27:0] x_out;
logic [26:0] y_out;
logic [20:0] vx_out, vy_out;

logic [31:0] vx_acc_in, vy_acc_in, vx_acc_out, vy_acc_out;//redundant, here so it compiles

assign x_in_b = x_out;
assign y_in_b = y_out;
assign vx_in_b = vx_out;
assign vy_in_b = vy_out;

assign x_in = x;
assign y_in = y;
assign vx_in = vx;
assign vy_in = vy;

initial begin
	clk = 0;
	en = 1;
	reset = 1'b1;
	#100
	reset = 1'b0;
	#50000
	$stop;
end

initial begin
	x = 0;
	y = 0;
	vx = 0;
	vy = 0;
end

always @(posedge clk) begin
	// num <= num + 4'd1;
end

/*

boid_accelerator dut(
	.*
);

*/



register_test_memory#(2) rtm( 
	.*
);

	
	logic [$clog2(2):0] which_boid;
	// 
	logic [6:0] 					 w_en;
	logic 							 dp_en;
	logic 							 r_en_tot;
	logic								 r_en_itr;

// notional dpath
always @(posedge clk) begin
	if (w_en != 0) begin
		x = x + 1;
		y = y + 1;
		vx = vx + 1;
		vy = vy + 1;
	end
end
	
xcel_ctrl#(2) ctrl(
	.*
);

logic [31:0] x_sh, y_sh;

assign x_sh = x >>> 16;
assign y_sh = y >>> 16;

/*

amax_bmin dut1 (
	.a(a_1),
	.b(b_1),
	.q(q_1)
);

fix15_mul dut2(
	.a(a_2),
	.b(b_2),
	.q(q_2)
);

*/

endmodule

/*

C model checking verilog helper modules against:

#include <stdio.h>

typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(( (((signed long long)(a)) << 15) / ((signed long long)(b))))

fix15 alpha_max_beta_min(fix15 a, fix15 b){
    fix15 max = (a > b) ? a : b;
    fix15 min = (a > b) ? b : a;
    
    return(max + (min >> 1));
}

int main(){
	// call alpha_max_beta_min on values
	// print input and output values out as hex
	// use those values to check against output from Verilog
}

*/