module boid_accelerator(
	input logic clk,
	input logic en,
	input logic reset,
	
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

module xy_sep_chk();

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

module xcel_ctrl
#(
	parameter num_boids = 2;
)
(
	input logic clk
);

localparam [2:0] 
	init 		= 4'd0,
	sa_init	= 4'd1,
	sa_ld 	= 4'd2,
	sa_calc	= 4'd3,
	ac_wb		= 4'd4;
	
	logic [1:0] state, next_state;
	
	logic [$clog2(num_boids+1):0] boid_itr_ctr, boid_tot_ctr; 
	
	// state update (+ counters)
	always @(posedge clk) begin
	
		// counter for per-boid loop
		if (state == sa_ld) begin
			boid_itr_ctr <= boid_itr_ctr + 1;
		end 
		else if (state == ac_wb | state == init)
			boid_itr_ctr <= 0;
		end
		
		// counter for total-boid loop
		if (state == ac_wb) begin
			boid_itr_ctr <= boid_itr_ctr + 1;
		end 
		else if (state == init)
			boid_itr_ctr <= 0;
		end
		
		state <= next_state;
	end
	
	// next-state logic
	always @(*) begin
		next_state = state;
		
		case(state)
			init:	begin
				next_state = !en ? sa_init : init;
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
				if (boid_tot_ctr >= (num_boids)) begin
					next_state = sa_init;
				end else begin
					next_state = init;
				end
			end
			
			default: // exhaustive case statement
		endcase
		
	end

	// output state control vars
	
	logic [$clog2(num_boids):0] which_boid;
	// 
	logic [6:0] 					 w_en;
	logic 							 r_en_tot;
	logic								 r_en_itr;
	
	// state control variables
	always @(*) begin
		w_en = 7'b0;
		r_en_tot = 1'b0;
		r_en_itr = 1'b0;
		case(state)
		init:
		sa_init: begin
			which_boid = boid_tot_ctr;
			r_en_tot = 1'b1;
		end
		sa_ld: begin
			// Must be reworked at parallelization step
			which_boid 
		sa_calc:
		ac_wb: w_en = 7'b0011111;
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
	
	// To investigate: making this a bi-directional bus
	
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



