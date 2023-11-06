module boid_accelerator(
	input logic clk,
	input logic dp_en,
	input logic reset,

	input logic [31:0]  
		x_in_b,  y_in_b, 	// current position
		px_in_b, py_in_b, 	// previous position (for erasure)
		vx_in_b, vy_in_b, 	// velocity
	
	
	output logic [31:0]  
		x,  y, 	// current position
		px, py, 	// previous position (for erasure)
		vx, vy 	// velocity
	
);

	// To future me: READ COMMENTS OF ctrl module IF RANDOM SHIT BREAKS
	// JUST READ COMMENTS TBH I WROTE PITFALLS ON COMMENTS FOR YOU

	/*
		This block retains state of x, y, vx, and vy. Every cycle it is clocked for, it
		will latch in updated values for x, y, vx, vy that should simply move the 
		
		ok this documentation is shit update it later. its just gonna move the boid
		around and make it turn at the boundary
		
		this added to make materially different commit
	*/
	
	// x/v transient wires
	
	logic [31:0] 			x_comb, 	y_comb;
	logic signed [31:0]	vx_comb,	vy_comb;
	

	// pos/speed regs for boid to be rewritten
	
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
	
	logic [31:0] 			xa_comb, 	ya_comb;
	logic [31:0] 			x_avg, 		y_avg;
	logic signed [31:0]	vxa_comb,	vya_comb;
	logic signed [31:0]	vx_avg,		vy_avg;
	
	// pos/speed average regs for accumulation
	
	d_reg #(32,0)
	x_avg_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? xa_comb : x_avg),
		.q			(x_avg)
	);
	
	d_reg #(32,0)
	y_avg_reg
	(	
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? ya_comb : y_avg),
		.q			(y_avg)
	);
	
	
	d_reg #(32,0)
	vx_avg_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? vxa_comb : vx_avg),
		.q			(vx_avg)
	);
	
	d_reg #(32,0)
	vy_avg_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? vya_comb : vy_avg),
		.q			(vy_avg)
	);
	
	logic [31:0] 			xc_comb, 	yc_comb;
	logic [31:0] 			x_close, 	y_close;
	
	// dx accumulator
	
	d_reg #(32,0)
	x_close_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? xc_comb : x_close),
		.q			(x_close)
	);
	
	d_reg #(32,0)
	y_close_reg
	(	
		.clk		(clk),
		.reset	(reset),
		.d			(en_post ? yc_comb : y_close),
		.q			(y_close)
	);
	
	// boid neighbor counter
	// should parametrize 
	
	logic [5:0] boid_ctr, boid_ctr_in;
	
	d_reg #(5, 0)
	boid_ctr_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d 		(boid_ctr_in),
		.q			(boid_ctr)
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
	
	
	logic signed [31:0] vx_bounded, vy_bounded;
	
	xy_bound_check xbc ( .* );
	
	//boundary enforcement
	
	/*
	
	logic [1:0] x_bchk, y_bchk;
	
	
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
	
	*/
	
	// speed enforcement
	
	logic signed [31:0] speed;
	
	amax_bmin speed_calc
	(
		.a		(vx_bounded),
		.b		(vy_bounded),
		.q		(speed)
	);
	
	logic [1:0] speed_bchk;
	assign speed_bchk = { ($signed(speed) > $signed(32'd8 << 16)), ($signed(speed) < $signed(32'd4 << 16)) };
	

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

module xy_sep_chk(

	input logic 	[31:0] x, y, x_in, y_in,
	
	input logic 	[31:0] vx, vy,
	
	input logic 	[31:0] x_avg, y_avg, vx_avg, vy_avg,
	
	input logic 	[31:0] x_close, y_close,
	
	output logic 	[31:0] xa_comb, ya_comb, vxa_comb, vya_comb,	
	
	output logic 	[31:0] xc_comb, yc_comb,
	
	input logic 	[5:0] boid_ctr,
	
	output logic 	[5:0] boid_ctr_in
);
	
	logic [31:0] x_sq, y_sq;
	
	logic [31:0] dx_t, dy_t;
	
	assign dx_t = x - x_in;
	
	assign dy_t = y - y_in;
	
	fix15_mul xmul (
		.a(dx_t),
		.b(dx_t),
		.q(x_sq)
	);
	
	fix15_mul ymul	(
		.a(dy_t),
		.b(dy_t),
		.q(y_sq)
	);	
	
	logic [31:0] d_sq;
	
	assign d_sq = x_sq + y_sq;
	
	logic [1:0] d_comparison;
	
	assign d_comparison = { (d_sq < (32'd64 << 16)); (d_sq < (32'd1600 << 16))};
	always @(*) begin
		
		case (d_comparison)
			01: begin 
				xa_comb = x_avg + x_in;
				ya_comb = y_avg + y_in;
				vxa_comb = vx_avg + vx_in;
				vya_comb = vy_avg + vy_in;
				boid_ctr_in = boid_ctr + 6'd1;
			end
			11: begin
				xc_comb = x_count + dx_t;
				yc_comb = y_count + dy_t;
			end
			default: begin
			// case 00 should have default behavior
				xa_comb = x_avg;
				ya_comb = y_avg;
				vxa_comb = vx_avg;
				vya_comb = vy_avg;
				xc_comb = x_count;
				yc_comb = y_count;
				boid_ctr_in = boid_ctr;
			end
			// 01 is an impossible case due to the nature of the conditionals,
			// if d_sq is less than 64, it is necessarily less than 1600
		endcase
	end
	
endmodule


module xy_bound_check
(
	// vx vy input
	input logic [31:0] vx, vy, x, y, x_bound, y_bound,
	
	// vx vy output
	output logic signed [31:0] vx_bounded, vy_bounded
);

	localparam signed [31:0] turnfactor = 32'h00001999;

	logic [1:0] x_bchk, y_bchk;
	
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

endmodule

// checked for 2 boids, should work
module xcel_ctrl
#(
	parameter num_boids = 2
)
(
	input logic clk,
	
	input logic en,
	
	input logic reset,
	
	// output state control vars
	
	output logic [$clog2(num_boids):0] 	which_boid,
	
	output logic [6:0] 					 	w_en,
	output logic 								dp_en,
	output logic 							 	r_en_tot,
	output logic							 	r_en_itr
);

	// negedge detector
	
	fall_edge_detector fed(
		.clk(clk),
		.signal(en),
		.q(en_post)
		);

localparam [2:0] 
	init 		= 4'd0,
	sa_init	= 4'd1,
	sa_ld 	= 4'd2,
	sa_calc	= 4'd3,
	ac_wb		= 4'd4;
	
	logic [2:0] state, next_state;
	
	logic [$clog2(num_boids+1):0] boid_itr_ctr, boid_tot_ctr; 
	
	// state update (+ counters)
	always @(posedge clk) begin
	
		// counter for per-boid loop
		if (state == sa_ld) begin
			boid_itr_ctr <= boid_itr_ctr + 1;
		end 
		else if (state == ac_wb | state == init) begin
			boid_itr_ctr <= 0;
		end
		
		// counter for total-boid loop
		if (state == ac_wb) begin
			boid_tot_ctr <= boid_tot_ctr + 1;
		end 
		else if (state == init) begin
			boid_tot_ctr <= 0;
		end
		
		state <= next_state;
	end
	
	// next-state logic
	always @(*) begin
		if (reset) begin
			next_state = init;
		end else begin
			next_state = state;
			case(state)
				init:	begin
					next_state = en_post ? sa_init : init;
				end
				sa_init: begin
					// this MUST be changed to stall until it accepts a valid from the memory system
					next_state = sa_ld;
				end
				sa_ld: begin
					// this MUST be changed to stall until it accepts a valid from the memory system
					next_state = sa_calc;
				end
				sa_calc: begin
					if (boid_itr_ctr >= (num_boids - 1)) begin
						next_state = ac_wb;
					end else begin
						next_state = sa_ld;
					end
				end
				ac_wb: begin
					if (boid_tot_ctr >= (num_boids - 1)) begin
						next_state = init;
					end else begin
						next_state = sa_init;
					end
				end
				
				//default: // exhaustive case statement
			endcase
		end
	end

	
	
	// state control variables
	always @(*) begin
		which_boid = 0;
		w_en = 7'b0;
		dp_en = 1'b0;
		r_en_tot = 1'b0;
		r_en_itr = 1'b0;
		case(state)
		init: begin // blank case, all defaults hold
		end
		sa_init: begin
			which_boid = boid_tot_ctr;
			r_en_tot = 1'b1;
		end
		sa_ld: begin
			// Must be reworked at parallelization step
			which_boid = !boid_tot_ctr;
			r_en_itr = 1'b1;
		end
		sa_calc: begin dp_en = 1'b1; end
		ac_wb: begin 
			which_boid = boid_tot_ctr;
			w_en = 7'b0011111; end
		endcase
	end

endmodule

module xcel_dp
();

endmodule

module register_test_memory
#(
	parameter num_boids = 2
)
(	
	input logic clk,
	input logic reset,
	input logic	[$clog2(num_boids):0] which_boid,
	
	input	logic	[6:0] 	w_en,
	
	// input ports
	
	input logic	[27:0] 	x_in,
	input logic	[26:0]	y_in,
	input	logic	[20:0]	vx_in,
	input	logic	[20:0] 	vy_in,
	
	input	logic	[31:0]	vx_acc_in,
	input logic	[31:0]	vy_acc_in,
	
	// output ports
	
	output logic	[27:0] 	x_out,
	output logic	[26:0]	y_out,
	output logic	[20:0]	vx_out,
	output logic	[20:0] 	vy_out,
	
	output logic	[31:0]	vx_acc_out,
	output logic	[31:0]	vy_acc_out
	
	// To investigate: making this a bi-directional bus
	
);
	logic [27:0] x_t 			[$clog2(num_boids):0];
	logic [26:0] y_t 			[$clog2(num_boids):0];
	logic [20:0] vx_t 		[$clog2(num_boids):0];
	logic [20:0] vy_t 		[$clog2(num_boids):0];
	logic [31:0] vx_acc_t 	[$clog2(num_boids):0]; 
	logic [31:0] vy_acc_t 	[$clog2(num_boids):0];
													
	// input genvar block, will instantiate test memory
						
	genvar i;
	generate for (i = 0; i < num_boids; i++) begin : tmem
		
		// each reg is muxed to state:
		
		d_reg #(28, ((28'd120 + 28'd40 * i) << 16)) x
		(	
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && w_en[1] && w_en[0]) ? x_in : x_t[i]),
			.q(x_t[i])
		);
		
		d_reg #(27, ((27'd120 + 27'd40 * i) << 16)) y
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && w_en[2] && w_en[0]) ? y_in : y_t[i]),
			.q(y_t[i])
		);
		
		d_reg #(21, ((28'd5) << 16)) vx
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && w_en[3] && w_en[0]) ? vx_in : vx_t[i]),
			.q(vx_t[i])
		);
		
		d_reg #(21, ((28'd4) << 16)) vy
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && w_en[4] && w_en[0]) ? vy_in : vy_t[i]),
			.q(vy_t[i])
		);
		
		d_reg #(32, (0)) x_a
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && w_en[5] && w_en[0]) ? vx_acc_in : vx_acc_t[i]),
			.q(vx_acc_t[i])
		);
		
		d_reg #(32, (0)) y_a
		(
			.clk(clk),
			.reset(reset),
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



