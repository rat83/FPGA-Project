module boid_accelerator(
	input logic clk,
	input logic reset,
	
	output logic [31:0]  
		x,  y, 	// current position
		px, py, 	// previous position (for erasure)
		vx, vy 	// velocity
	
);
	/*
		This block retains state of x, y, vx, and vy. Every cycle it is clocked for, it
		will latch in updated values for x, y, vx, vy that should simply move the 
		
		ok this documentation is shit update it later. its just gonna move the boid
		around and make it turn at the boundary
	*/

	// x/v transient wires
	
	logic [31:0] x_comb, 	y_comb
					 xv_comb,	yv_comb;
	

	// pos/speed regs
	
	d_reg x_reg #(32,(32'd180 << 16))
	(
		.clk		(clk),
		.reset	(reset),
		.d			(x_comb),
		.q			(x)
	);
	
	d_reg y_reg #(32,(32'd240 << 16))
	(	
		.clk		(clk),
		.reset	(reset),
		.d			(y_comb),
		.q			(y)
	);
	
	
	d_reg vx_reg #(32,(32'd4 << 16))
	(
		.clk		(clk),
		.reset	(reset),
		.d			(vx_comb),
		.q			(vx)
	);
	
	d_reg vy_reg #(32,(32'd4 << 16))
	(
		.clk		(clk),
		.reset	(reset),
		.d			(vx_comb),
		.q			(vx)
	);
	
	// margins
	
	logic [31:0] x_bound, y_bound;
	
	d_reg x_bound_reg #(32,(32'd100 << 16))
	(
		.clk		(clk),
		.reset	(reset),
		.d 		(x_bound),
		.q			(x_bound)
	);
	
	d_reg y_bound_reg	#(32,(32'd100 << 16))
	(
		.clk		(clk),
		.reset	(reset),
		.d 		(y_bound),
		.q			(y_bound)
	);
	
	logic [31:0] speed;
	logic [31:0] vx_sq, vy_sq;
	
	// speed calculation
	
	fix15_mul vx_mul
	(
		.a		(vx),
		.b		(vx),
		.q		(vx_sq)
	);
	
	fix15_mul vy_mul
	(
		.a		(vy),
		.b		(vy),
		.q		(vy_sq)
	);
	
	amax_bmin speed_calc
	(
		.a		(vx_sq),
		.b		(vy_sq),
		.q		(speed)
	);
	
	
	
endmodule

module d_reg
	#(
		parameter d_width = 32,
		parameter d_res_v = 0
	)(
	input logic 	clk,
	input logic 	reset,
	input logic 	[d_width-1:0] d,
	output logic 	[d_width-1:0] q
	);
	
	always @(posedge clk) begin
		if (reset) begin
			q <= d_res_v;
		end else begin
			q <= d;
		end
	end
	
endmodule

module fix15_mul(
	input logic 	[31:0] a, b,
	output logic	[31:0] q
	);
	logic [63:0] temp;
	assign temp = a * b;
	assign q = temp[47:15];
	
endmodule

module amax_bmin(
	input logic 	[31:0] a, b,
	output logic	[31:0] q
	);
	
	logic [31:0] a_temp, b_temp;
	
	logic [31:0] alpha, beta;
	
	//absolute value
	always_comb begin
		if (a[31] == 1) begin
			a_temp = -a;
		end else begin
			a_temp = a;
		end
		
		if (b[31] == 1) begin
			b_temp = -b;
		end else begin
			b_temp = b;
		end		
		
		if (a > b) begin
			alpha = a;
			beta = b;
		end else begin
			alpha = b;
			beta = a;
		end
		
		q = alpha + (beta >> 1);
		
	end
	
	
endmodule
	
	
