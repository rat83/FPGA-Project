`timescale 1ns/1ns

module testbench();

logic clk, reset, en;

always begin
	#10
	clk = !clk;
end

always begin
	#90
	en = ~en;
	#10
	en = ~en;
end

logic [31:0] a_1, b_1, c_1, q_1, a_2, b_2, c_2, q_2;

logic [31:0] x, y, vx, vy, px, py;

// I'm aware this testbench is kinda gross right now, these are mostly 'sanity check'
// type tests to make sure the modules I instantiated actually work properly

// checking amax_bmin
/*
initial begin
	a_1 = 32'h0000c000;
	b_1 =	32'h00013333;
	c_1 =	32'h00019333;
	#50
	assert (c_1 == q_1);
	a_1 = 32'h0001d999;
	b_1 = 32'h00029999;
	c_1 = 32'h00038665;
	#50
	assert (c_1 == q_1);
	a_1 = 32'h0004f333;
	b_1 = 32'h00014000;
	c_1 = 32'h00059333;
	#50
	assert (c_1 == q_1);
	a_1 = 32'h011c10e6;
   b_1 = 32'h00de1f1b;
   c_1 = 32'h018b2073;
   #50;
	assert (c_1 == q_1);
	a_1 = 32'h0071244e;
   b_1 = 32'h0000004b;
   c_1 = 32'h00712473;
   #50;
   assert (c_1 == q_1);
	a_1 = 32'h00566666;
   b_1 = 32'h09c3f330;
   c_1 = 32'h09ef2663;
   #50;
   assert (c_1 == q_1);
end

// checking fix15_mul
initial begin
	a_2 = 32'h00566666;
   b_2 = 32'h09c3f330;
   c_2 = 32'h97775230;
   #50;
	assert (c_2 == q_2);
	a_2 = 32'h00000013;
   b_2 = 32'h01199c28;
   c_2 = 32'h000029cd;
   #50;
   assert (c_2 == q_2);
	a_2 = 32'hffffffff;
	b_2 = 32'hffffffff;
	c_2 = 32'hFFFC0000;
   #50;
   assert (c_2 == q_2);
	
end
*/

initial begin
	clk = 0;
	en = 1;
	reset = 1'b1;
	#100
	reset = 1'b0;
	#5000
	$stop;
end

always @(posedge clk) begin
	// num <= num + 4'd1;
end

boid_accelerator dut(
	.*
);

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