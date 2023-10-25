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
	
	localparam signed [31:0] turnfactor = 32'h00001999;
	
	// x/v transient wires
	
	logic [31:0] 			x_comb, 	y_comb;
	logic signed [31:0]	vx_comb,	vy_comb;
	
	// negedge detector
	
	fall_edge_detector fed(
		.clk(clk),
		.signal(en),
		.q(en_post)
		);

	// pos/speed regs
	
	d_reg #(32,((32'd115) << 16))
	x_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? x_comb : x),
		.q			(x)
	);
	
	d_reg #(32,((32'd319) << 16))
	y_reg
	(	
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? y_comb : y),
		.q			(y)
	);
	
	
	d_reg #(32,((1 * 32'd4) << 16))
	vx_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? vx_comb : vx),
		.q			(vx)
	);
	
	d_reg #(32,((1 * 32'd4) << 16))
	vy_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? vy_comb : vy),
		.q			(vy)
	);
	
	// margins
	
	logic [31:0] x_bound, y_bound;
	
	d_reg #(32,((32'd100 << 16)))
	x_bound_reg 
	(
		.clk		(clk),
		.reset	(reset),
		.d 		(x_bound),
		.q			(x_bound)
	);
	
	d_reg #(32,((32'd100 << 16)))
	y_bound_reg	
	(
		.clk		(clk),
		.reset	(reset),
		.d 		(y_bound),
		.q			(y_bound)
	);
	
	
	
	//boundary enforcement
	
	
	logic [1:0] x_bchk, y_bchk;
	
	logic signed [31:0] vx_bounded, vy_bounded;
	
	logic signed [31:0] x_max_b_t, y_max_b_t;
	
	assign x_max_b_t = $signed((32'd640 << 16) - x_bound);
	assign y_max_b_t = $signed((32'd480 << 16) - y_bound);
	
	assign x_bchk = {$signed (x) > $signed(x_max_b_t), $signed(x) < $signed(x_bound)};
	assign y_bchk = {$signed (y) > $signed(y_max_b_t), $signed(y) < $signed(y_bound)};
	
	always_comb begin
		case (x_bchk)
			2'd0: begin
					vx_bounded = vx;
				end
			2'd1: begin
					vx_bounded = vx + turnfactor;
				end
			2'd2: begin
					vx_bounded = vx - turnfactor;
				end
			default: begin
					vx_bounded = vx;
					// 3 is unreachable, contradictory state
				end 
		endcase
		
		case (y_bchk)
			2'd0: begin
					vy_bounded = vy;
				end
			2'd1: begin
					vy_bounded = vy + turnfactor;
				end
			2'd2: begin
					vy_bounded = vy - turnfactor;
				end
			default: begin
					vy_bounded = vy;
					// 3 is unreachable, contradictory state
				end 
		endcase
	end
	
	logic signed [31:0] speed;
	logic signed [31:0] vx_sq, vy_sq;
	
	// speed maximum enforcement
	
	/*
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
	*/
	
	amax_bmin speed_calc
	(
		.a		(vx_bounded),
		.b		(vy_bounded),
		.q		(speed)
	);
	
	logic [1:0] speed_bchk;
	assign speed_bchk = { ($signed(speed) > $signed(32'd8 << 16)), ($signed(speed) < $signed(32'd4 << 16)) };
	
	// speed enforcement
	always_comb begin
		case (speed_bchk)
			2'd0: begin
					vx_comb = vx_bounded;
					vy_comb = vy_bounded;
				end
			2'd1: begin
					vx_comb = vx_bounded + (vx_bounded >>> 2 + 32'b1);
					vy_comb = vy_bounded + (vy_bounded >>> 2 + 32'b1);
				end
			2'd2: begin
					vx_comb = vx_bounded - (vx_bounded >>> 2);
					vy_comb = vy_bounded - (vy_bounded >>> 2);
				end
			default: begin
					vx_comb = vx_bounded;
					vy_comb = vy_bounded;
					// 3 is unreachable, contradictory state
				end 
		endcase
	end
	
	assign x_comb = x + vx_comb;
	assign y_comb = y + vy_comb;
	
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
	input logic signed 	[31:0] a, b,
	output logic signed	[31:0] q
	);
	
	logic signed [63:0] temp;
	assign temp = a * b;
	assign q = temp >>> 15;
	
endmodule

module amax_bmin(
	input logic 	[31:0] a, b,
	output logic	[31:0] q
	);
	
	logic signed [31:0] 	a_temp, 	b_temp,
								a_out,	b_out;
	
	logic signed [31:0] alpha, beta;
	
	fix15_mul a_mul(
		.a(a),
		.b(32'hffff0000),
		.q(a_out)
		);
		
	fix15_mul b_mul(
		.a(32'hffff0000),
		.b(b),
		.q(b_out)
		);
	
	
	
	//absolute value
	always_comb begin
		if (a[31] == 1) begin
			a_temp = a_out + (32'b1);
		end else begin
			a_temp = a;
		end
		
		if (b[31] == 1) begin
			b_temp = b_out + (32'b1);
		end else begin
			b_temp = b;
		end		
		
		if (a > b) begin
			alpha = a_temp;
			beta = b_temp;
		end else begin
			alpha = b_temp;
			beta = a_temp;
		end
		
		q = alpha + (beta >>> 1);
		
	end
	
	
endmodule

// fall_edge_detector will pulse 1 cycle high when a falling edge is recorded (with 1 cycle delay)

module fall_edge_detector(
  input  logic clk, signal,
  output logic q
);

  logic signalPrev;

  always_ff @(posedge clk) begin
    signalPrev <= signal;
    q       <= (!signal && signalPrev);
  end

endmodule

// zero_pad_fix15 will sign extend the variable on fix_in to 32 bits, with 
// compile-time defined parameters to determine how it should be interpreted.
// on a [total_bit_width - 1:0] register, the variable to be sign extended
// will be on bits [ (16 + fix_whole_bit_width + lsb_offset) : lsb_offset]
// and the sign will be on bit 16 + fix_whole_bit_width + lsb_offset.
// Requirement: total_bit_width, as defined, must be equal to or greater than 
// 16 + fix_whole_bit_width + lsb_offset.

module zero_pad_fix15
#(	
	parameter fix_whole_bit_width = 16,
	parameter lsb_offset				= 0,
	parameter total_bit_width		= 32
)
(	
	input 	[total_bit_width - 1:0] fix_in,
	output	[31:0]						fix_out
);
	localparam input_fix_bit_width = 15 + fix_whole_bit_width;
	assign fix_out = 
		{{(32 - input_fix_bit_width + lsb_offset){fix_in[input_fix_bit_width]}}, fix_in[input_fix_bit_width - 1:lsb_offset] };

endmodule

module register_test_memory
#(
	parameter num_boids = 2
)
(
	input 	[$clog2(num_boids):0]	which_boid,
	
	input		[6:0] 	w_en,
	
	// input ports
	
	input 	[27:0] 	x_in,
	input 	[26:0]	y_in,
	input		[20:0]	vx_in,
	input		[20:0] 	vy_in,
	
	input		[31:0]	vx_acc_in,
	input 	[31:0]	vy_acc_in,
	
	// output ports
	
	output 	[27:0] 	x_out,
	output 	[26:0]	y_out,
	output	[20:0]	vx_out,
	output	[20:0] 	vy_out,
	
	output	[31:0]	vx_acc_out,
	output 	[31:0]	vy_acc_out
);
	logic [$clog2(num_boids):0] x_t 			[27:0];
	logic [$clog2(num_boids):0] y_t			[26:0];
	logic [$clog2(num_boids):0] vx_t 		[20:0];
	logic [$clog2(num_boids):0] vy_t 		[20:0];
	logic [$clog2(num_boids):0] vx_acc_t	[31:0]; 
	logic [$clog2(num_boids):0] vy_acc_t	[31:0];
													
	// input genvar block, will instantiate test memory
						
	genvar i;
	generate for (i = 0; i < num_boids; i++) begin : tmem
		
		// each reg is muxed to state:
		
		d_reg #(28, ((28'd120 + 28'd40 * i) << 16)) x
		(
			.d((which_boid == i && w_en[1] && w_en[0]) ? x_in : x_t[i]),
			.q(x_t[i])
		);
		
		d_reg #(27, ((27'd120 + 27'd40 * i) << 16)) y
		(
			.d((which_boid == i && w_en[2] && w_en[0]) ? y_in : y_t[i]),
			.q(y_t[i])
		);
		
		d_reg #(21, ((28'd5) << 16)) vx
		(
			.d((which_boid == i && w_en[3] && w_en[0]) ? vx_in : vx_t[i]),
			.q(vx_t[i])
		);
		
		d_reg #(21, ((28'd4) << 16)) vy
		(
			.d((which_boid == i && w_en[4] && w_en[0]) ? vy_in : vy_t[i]),
			.q(vy_t[i])
		);
		
		d_reg #(32, (0)) x_a
		(
			.d((which_boid == i && w_en[5] && w_en[0]) ? vx_acc_in : vx_acc_t[i]),
			.q(vx_acc_t[i])
		);
		
		d_reg #(32, (0)) y_a
		(
			.d((which_boid == i && w_en[6] && w_en[0]) ? vy_acc_in : vy_acc_t[i]),
			.q(vy_acc_t[i])
		);
		end
	endgenerate
	
	// output mux
	
	assign x_out 		= 		  x_t[which_boid];
	assign y_out 		= 		  y_t[which_boid];
	assign vx_out		= 		 vx_t[which_boid];
	assign vy_out		= 		 vy_t[which_boid];
	assign vx_acc_out	=	vx_acc_t[which_boid];
	assign vy_acc_out = 	vy_acc_t[which_boid];
	
endmodule



