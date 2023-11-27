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
	
	output logic [$clog2(num_boids)-1:0] 	which_boid,
	
	output logic [6:0] 					 	wb_en,
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
	init 		= 3'd0,
	sa_init	= 3'd1,
	sa_ld 	= 3'd2,
	sa_calc	= 3'd3,
	ac_wb		= 3'd4;
	
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
		wb_en = 7'b0;
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
		end
		sa_calc: begin 
			which_boid = !boid_tot_ctr;
			r_en_itr = 1'b1; 
		end
		ac_wb: begin 
			which_boid = boid_tot_ctr;
			wb_en = 7'b0011111; end
		endcase
	end

endmodule


