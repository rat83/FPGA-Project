module boid_accelerator(
	input logic clk,
	input logic en,
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
		
		this added to make materially different commit
	*/
	
	localparam [31:0] turnfactor = 32'h00001999;
	
	// x/v transient wires
	
	logic [31:0] x_comb, 	y_comb,
					 xv_comb,	yv_comb;
	

	// pos/speed regs
	
	d_reg #(32,(32'd180 << 16))
	x_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(x_comb),
		.q			(x)
	);
	
	d_reg #(32,(32'd240 << 16))
	y_reg
	(	
		.clk		(clk),
		.reset	(reset),
		.d			(y_comb),
		.q			(y)
	);
	
	
	d_reg #(32,(32'd4 << 16))
	vx_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(vx_comb),
		.q			(vx)
	);
	
	d_reg #(32,(32'd4 << 16))
	vy_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(vy_comb),
		.q			(vy)
	);
	
	// margins
	
	logic [31:0] x_bound, y_bound;
	
	d_reg #(32,(32'd100 << 16))
	x_bound_reg 
	(
		.clk		(clk),
		.reset	(reset),
		.d 		(x_bound),
		.q			(x_bound)
	);
	
	d_reg #(32,(32'd100 << 16))
	y_bound_reg	
	(
		.clk		(clk),
		.reset	(reset),
		.d 		(y_bound),
		.q			(y_bound)
	);
	
	
	
	//boundary enforcement
	
	
	logic [1:0] x_bchk, y_bchk;
	
	logic [31:0] vx_bounded, vy_bounded;
	
	assign x_bchk = {x > (640 - x_bound), x < x_bound};
	assign y_bchk = {y > (480 - y_bound), y < y_bound};
	
	case (x_bchk)
		2'd0: begin
				vx_bounded = vx;
			end
		2'd1: begin
				vx_bounded = vx + turnfactor;
			end
		2'd2: begin
				vx_bounded = vx - turnfactor
			end
		default: begin
				vx_bounded = vx;
				// 3 is unreachable, contradictory state
			end 
	endcase
	
	case (x_bchk)
		2'd0: begin
				vy_bounded = vy;
			end
		2'd1: begin
				vy_bounded = vy + turnfactor;
			end
		2'd2: begin
				vy_bounded = vy - turnfactor
			end
		default: begin
				vy_bounded = vy;
				// 3 is unreachable, contradictory state
			end 
	endcase
	
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
	
	assign logic [1:0] speed_bchk = { (speed > (32'd8 << 16)), (speed < (32'd4 << 16)) };
	
	// speed enforcement
	
	case (speed_bchk)
		2'd0: begin
				vx_comb = vx_bounded;
				vy_comb = vy_bounded;
			end
		2'd1: begin
				vx_comb = vx_bounded + (vx_bounded >> 2);
				vy_comb = vy_bounded + (vy_bounded >> 2);
			end
		2'd2: begin
				vx_comb = vx_bounded - (vx_bounded >> 2);
				vy_comb = vy_bounded - (vy_bounded >> 2);
			end
		default: begin
				vx_comb = vx_bounded;
				vy_comb = vy_bounded;
				// 3 is unreachable, contradictory state
			end 
	endcase
	
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

module fall_edge_detector
	
	
