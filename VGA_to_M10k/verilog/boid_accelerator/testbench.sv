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
	#20
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

logic [31:0] x_in_32; 
logic [31:0] y_in_32; 
logic [31:0] vx_in_32, vy_in_32;

logic [31:0] x_out_32;
logic [31:0] y_out_32;
logic [31:0] vx_out_32, vy_out_32;

assign x_in_b = x_out;
assign y_in_b = y_out;
assign vx_in_b = vx_out;
assign vy_in_b = vy_out;

assign x_in_32 = x;
assign y_in_32 = y;
assign vx_in_32 = vx;
assign vy_in_32 = vy;

initial begin
	clk = 0;
	en = 1;
	reset = 1'b1;
	#100
	reset = 1'b0;
	#50000
	$stop;
end

	logic is_boid_here;
	
	boid_accelerator xcel
	(
		.*
	);
	
	initial begin
		x = 105;
		y = 105;
		
		
	end
	
	/*

	// d-path combined verification
	
	logic r_en_tot, r_en_itr;
	
	logic [6:0] wb_en;
	
	logic [31:0]
		x_in_xcel,
		y_in_xcel,
		vx_in_xcel,
		vy_in_xcel;
		
	logic [31:0]
		x_out_xcel,
		y_out_xcel,
		vx_out_xcel,
		vy_out_xcel;
	
	
	
	xcel_dp xdp
	(
		.*
		
	);
	


	
	
	initial begin
	
		r_en_tot = 0;
		r_en_itr = 0;
		wb_en		= 0;
		x_in_xcel 	= 105 << 16;
		y_in_xcel 	= 105 << 16;
		vx_in_xcel 	= 2 << 16;
		vy_in_xcel 	= -1 << 16;
		
		# 100
		
		r_en_tot = 1;
		
		#20
		
		r_en_tot = 0;
		
		#80
		
		r_en_itr = 1;
		
		x_in_xcel 	= 115 << 16;
		y_in_xcel 	= 115 << 16;
		vx_in_xcel 	= 3 << 16;
		vy_in_xcel 	= 0 << 16;
		
		#20
		
		r_en_itr = 0;
		
		#80
		
		wb_en = 1;
		
		#20
		
		wb_en = 0;
		
		//#100
		
		
	end
	*/
	/*
	
	// xy_writeback verification
	
	// inputs
	logic [31:0] x_bound, y_bound;
	
	assign x_bound = 100 << 16, y_bound = 100 << 16;
	
	logic [31:0] x_avg, y_avg;
	
	logic signed [31:0] vx_avg, vy_avg;
	
	logic [31:0] x_close, y_close;
	
	logic [5:0] boid_ctr;
	
	// vx vy output
	logic signed [31:0] vx_bounded, vy_bounded; 

	initial begin
		x = 0;
		y = 0;
		vx = 0;
		vy = 0;
		x_avg = 0;
		y_avg = 0;
		vx_avg = 0;
		vy_avg = 0;
		x_close = 0;
		y_close = 0;
		boid_ctr = 0;
		
		#100
		
		x = 150 << 16;
		y = 150 << 16;
		vx = 3 << 16;
		vy = 0 << 16;
		
		x_avg = 155 << 16;
		y_avg = 145 << 16;
		
		vx_avg = 2 << 16;
		vy_avg = 3 << 16;
		
		x_close = 0;
		y_close = 0;
		
		boid_ctr = 1;
		
	end
	
	
	xy_writeback xyw (.*);
	
	*/
	
	/*

	// xy_sep_chk verification

	// input

	logic 	[31:0] x_in_xcel, y_in_xcel;
	
	logic 	[31:0] vx_in_xcel, vy_in_xcel;
	
	logic 	[31:0] x_avg, y_avg;
	
	logic 	[31:0] x_close, y_close;
	
	logic signed 	[31:0] vx_avg, vy_avg;
	
	logic 	[5:0] boid_ctr;
		
	// output

	logic 	[31:0] xa_comb, ya_comb;
	
	logic signed 	[31:0] vxa_comb, vya_comb;
	
	logic 	[31:0] xc_comb, yc_comb;
	
	logic 	[5:0] boid_ctr_in;

//pseudo memory
	
always @(posedge clk) begin
	if(~reset) begin
		boid_ctr = boid_ctr_in;
		x_close = xc_comb;
		y_close = yc_comb;
		x_avg = xa_comb;
		y_avg = ya_comb;
		vx_avg = vxa_comb;
		vy_avg = vya_comb;
	end
end
	
	initial begin
		x = 0;
		y = 0;
		x_in_xcel = 0;
		y_in_xcel = 0;
		vx = 0;
		vy = 0;
		vx_in_xcel = 0;
		vy_in_xcel = 0;
		x_avg = 0;
		y_avg = 0;
		x_close = 0;
		y_close = 0;
		vx_avg = 0;
		vy_avg = 0;
		boid_ctr = 0;
		
		#100
		
		x = 140 << 16;
		y = 140 << 16;
		vx = 3 << 16;
		vy = 3 << 16;
		
		x_in_xcel = 150 << 16;
		y_in_xcel = 140 << 16;
		vx_in_xcel = 3 << 16;
		vy_in_xcel = 3 << 16;
		
		#100 
		
		x_in_xcel = 140 << 16;
		y_in_xcel = 150 << 16;
		vx_in_xcel = -3 << 16;
		vy_in_xcel = 3 << 16;
				
		#100 
		
		x_in_xcel = 150 << 16;
		y_in_xcel = 150 << 16;
		vx_in_xcel = 3 << 16;
		vy_in_xcel = -3 << 16;
				
		#100 
		
		x_in_xcel = 180 << 16;
		y_in_xcel = 180 << 16;
		vx_in_xcel = 3 << 16;
		vy_in_xcel = 3 << 16;
		
		#100 
		
		x_in_xcel = 150 << 16;
		y_in_xcel = 150 << 16;
		vx_in_xcel = -3 << 16;
		vy_in_xcel = 3 << 16;
		
		#100 
		
		x_in_xcel = 141 << 16;
		y_in_xcel = 141 << 16;
		vx_in_xcel = 3 << 16;
		vy_in_xcel = 3 << 16;
		
		#100
		
		x_in_xcel = 140 << 16;
		y_in_xcel = 150 << 16;
		vx_in_xcel = -3 << 16;
		vy_in_xcel = 3 << 16;
		
		#1000
		
		x_in_xcel = 141 << 16;
		y_in_xcel = 141 << 16;
		vx_in_xcel = 3 << 16;
		vy_in_xcel = 3 << 16;
		
		
	end

	xy_sep_chk xsc (.*);
	*/


// memory verification
/*
logic [31:0] x_chk_in, y_chk_in;

logic is_boid_here;

initial begin
	x = 0;
	y = 0;
	vx = 0;
	vy = 0;
	x_chk_in = 1;
	y_chk_in = 1;
end


register_test_mem_wrapper#(2) rtm( 
	.*
);

	
	logic [$clog2(2):0] which_boid;
	// 
	logic [6:0] 					 wb_en;
	logic 							 dp_en;
	logic 							 r_en_tot;
	logic								 r_en_itr;

// notional dpath
always @(posedge clk) begin
	if (wb_en != 0) begin
		x = x + 1;
		y = y + 1;
		vx = vx + 1;
		vy = vy + 1;
	end
	
end
	
xcel_ctrl#(2) ctrl(
	.*
);

// xcel_dpath dp( .* );

logic [31:0] x_sh, y_sh;

assign x_sh = x >>> 16;
assign y_sh = y >>> 16;

*/
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