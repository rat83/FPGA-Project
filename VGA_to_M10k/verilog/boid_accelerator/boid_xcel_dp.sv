module xcel_dp(
	input logic clk,
	input logic reset,
	input logic [9:0] SW,
	
	// control signals
	input logic r_en_tot, // en_write
	
	input logic r_en_itr, // en_itr
	
	input logic [6:0] wb_en,
	
	// data from memory
	input logic signed [31:0]
		x_in_xcel,
		y_in_xcel,
		vx_in_xcel,
		vy_in_xcel,
		
	output logic [31:0]
		x_out_xcel,
		y_out_xcel,
		vx_out_xcel,
		vy_out_xcel
);

	// To future me: READ COMMENTS OF ctrl module IF RANDOM SHIT BREAKS
	// JUST READ COMMENTS TBH I WROTE PITFALLS ON COMMENTS FOR YOU

	/*
		This block retains state of x, y, vx, and vy. Every cycle it is clocked for, it
		will latch in updated values for x, y, vx, vy that should simply move the 
		
		ok this documentation is shit update it later. its just gonna move the boid
		around and make it turn at the boundary
		
		*_comb values are the finalized combinatorial value of some variable prior to
		it being sent to a register or memory
	*/
	
	// x/v transient wires
	// these no longer are written back into 
	
	logic signed [31:0] 	x, 	y;
	logic signed [31:0]	vx,	vy;
	
	logic signed [31:0] 	x_comb, 	y_comb;
	logic signed [31:0]	vx_comb,	vy_comb;
	
	
	// pos/speed regs for boid to be rewritten
	
	d_reg #(32,0)
	x_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(r_en_tot ? x_in_xcel : x),
		.q			(x)
	);
	
	d_reg #(32,0)
	y_reg
	(	
		.clk		(clk),
		.reset	(reset),
		.d			(r_en_tot ? y_in_xcel : y),
		.q			(y)
	);
	
	
	d_reg #(32,0)
	vx_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(r_en_tot ? vx_in_xcel : vx),
		.q			(vx)
	);
	
	d_reg #(32,0)
	vy_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(r_en_tot ? vy_in_xcel : vy),
		.q			(vy)
	);
	
	logic signed [31:0] 	xa_comb, 	ya_comb;
	logic signed [31:0] 	x_avg, 		y_avg;
	logic signed [31:0]	vxa_comb,	vya_comb;
	logic signed [31:0]	vx_avg,		vy_avg;
	
	// pos/speed average regs for accumulation
	
	d_reg #(32,0)
	x_avg_reg
	(
		.clk		(clk),
		.reset	(reset),
		.d			(r_en_tot ? 0 : r_en_itr ? xa_comb : x_avg),
		.q			(x_avg)
	);
	
	d_reg #(32,0)
	y_avg_reg
	(	
		.clk		(clk),
		.reset	(reset),
		.d			(r_en_tot ? 0 : r_en_itr ? ya_comb : y_avg),
		.q			(y_avg)
	);
	
	
	d_reg #(32,0)
	vx_avg_reg
	(
		.clk		(clk),
		.reset	(reset | r_en_tot),
		.d			(r_en_tot ? 0 : r_en_itr ? vxa_comb : vx_avg),
		.q			(vx_avg)
	);
	
	d_reg #(32,0)
	vy_avg_reg
	(
		.clk		(clk),
		.reset	(reset | r_en_tot),
		.d			(r_en_tot ? 0 : r_en_itr ? vya_comb : vy_avg),
		.q			(vy_avg)
	);
	
	logic signed [31:0] 	xc_comb, 	yc_comb;
	logic signed [31:0] 	x_close, 	y_close;
	
	// dx accumulator
	
	d_reg #(32,0)
	x_close_reg
	(
		.clk		(clk),
		.reset	(reset | r_en_tot),
		.d			(r_en_itr ? xc_comb : x_close),
		.q			(x_close)
	);
	
	d_reg #(32,0)
	y_close_reg
	(	
		.clk		(clk),
		.reset	(reset | r_en_tot),
		.d			(r_en_itr ? yc_comb : y_close),
		.q			(y_close)
	);
	
	// boid neighbor counter
	// should parametrize 
	
	logic [5:0] boid_ctr, boid_ctr_in;
	
	d_reg #(6, 0)
	boid_ctr_reg
	(
		.clk		(clk),
		.reset	(reset | r_en_tot),
		.d 		(r_en_tot ? 0 : r_en_itr ? boid_ctr_in : boid_ctr),
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
	
	// Accumulation pipeline
	
	xy_sep_chk xsc ( .* );
	
	logic signed [31:0] vx_bounded, vy_bounded;

	// Writeback pipeline variables
	
	logic signed [31:0] 		vx_wb, vy_wb, x_wb, y_wb;
	
	logic signed [31:0] 		x_avg_wb, y_avg_wb;
	
	logic signed [31:0] 		vx_avg_wb, vy_avg_wb;
	
	logic signed [31:0] 		x_close_wb, y_close_wb;
	
	logic [5:0] 				boid_ctr_wb;
	
	// writeback input muxing
	
	// eliminates switching in writeback pipeline prior to the data being
	// ready to be written back to memory
	
	assign x_wb 			= x;//wb_en[0] ? x : 32'b0;
	
	assign y_wb 			= y;//wb_en[0] ? y : 32'b0;
	
	assign vx_wb 			= vx;//wb_en[0] ? vx : 32'b0;
	
	assign vy_wb 			= vy;//wb_en[0] ? vy : 32'b0;
	
	assign x_avg_wb 		= wb_en[0] ? x_avg : 32'b0;
	
	assign y_avg_wb 		= wb_en[0] ? y_avg : 32'b0;
	
	assign vx_avg_wb 		= wb_en[0] ? vx_avg : 32'b0;
	
	assign vy_avg_wb 		= wb_en[0] ? vy_avg : 32'b0;
	
	assign x_close_wb 	= wb_en[0] ? x_close : 32'b0;

	assign y_close_wb 	= wb_en[0] ? y_close : 32'b0;
	
	assign boid_ctr_wb 	= wb_en[0] ? boid_ctr : 6'b0;
	
	// writeback module
	
	xy_writeback xbc ( .* );
	
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
	
	assign x_comb = x_wb + vx_comb;
	assign y_comb = y_wb + vy_comb;
	
	// Outputs assigned from comb to output ports
	
	assign x_out_xcel = x_comb;
	assign y_out_xcel = y_comb;
	assign vx_out_xcel = vx_comb;
	assign vy_out_xcel = vy_comb;
	
endmodule

module xy_sep_chk(

	input logic signed 	[31:0] x, y, x_in_xcel, y_in_xcel,
	
	input logic signed	[31:0] vx, vy, vx_in_xcel, vy_in_xcel,
	
	input logic signed 	[31:0] x_avg, y_avg, 
	
	input logic signed 	[31:0] x_close, y_close,
	
	output logic signed 	[31:0] xa_comb, ya_comb, 
	
	input logic signed 	[31:0] vx_avg, vy_avg,
		
	output logic signed 	[31:0] vxa_comb, vya_comb,
	
	output logic signed 	[31:0] xc_comb, yc_comb,
	
	input logic 	[5:0] boid_ctr,
	
	output logic 	[5:0] boid_ctr_in
);
	
	logic [31:0] x_sq, y_sq;
	
	logic [31:0] dx_t, dy_t;
	
	assign dx_t = x - x_in_xcel;
	
	assign dy_t = y - y_in_xcel;
	
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
	
	logic d01_a, d01_b;
	
	localparam fix15_1600 = 32'h06400000;
	//                      fix15 1600  
	assign d01_a = (d_sq < fix15_1600);
	
	assign d01_b = (boid_ctr != 6'b111111);
	
								
	assign d_comparison = { (d_sq < (32'd64 << 16)), d01_a && d01_b};
	// { distance less than 'avoid threshold' ; distance less than 'visual' threshold and fewer than 31 boids have been collected }
	
	always @(*) begin	
		casez (d_comparison)
			2'b1?: begin
				xa_comb = x_avg;
				ya_comb = y_avg;
				vxa_comb = vx_avg;
				vya_comb = vy_avg;
				xc_comb = x_close + dx_t;
				yc_comb = y_close + dy_t;
				boid_ctr_in = boid_ctr;
			end
			2'b01: begin 
				xa_comb = x_avg + x_in_xcel;
				ya_comb = y_avg + y_in_xcel;
				vxa_comb = vx_avg + vx_in_xcel;
				vya_comb = vy_avg + vy_in_xcel;
				xc_comb = x_close;
				yc_comb = y_close;
				boid_ctr_in = boid_ctr + 6'd1;
			end
			default: begin
			// case 00 should have default behavior
				xa_comb = x_avg;
				ya_comb = y_avg;
				vxa_comb = vx_avg;
				vya_comb = vy_avg;
				xc_comb = x_close;
				yc_comb = y_close;
				boid_ctr_in = boid_ctr;
			end
			// 10 arises if we have detected 31 neighboring boids, but we want to allow
			// boids that are too close to be collected for separation + we will operate 
			// in lockstep for multicore in the future regardless
		endcase
	end
	
endmodule


module xy_writeback
(
	// vx vy input
	input logic signed [31:0] vx_wb, vy_wb, x_wb, y_wb,
	
	input logic signed [31:0] x_avg_wb, y_avg_wb,
	
	input logic signed [31:0] vx_avg_wb, vy_avg_wb,
	
	input logic signed [31:0] x_close_wb, y_close_wb,
	
	input logic [31:0]  		  x_bound, y_bound,
	
	input logic [5:0] boid_ctr_wb,
	input logic [9:0] SW,
	
	// vx vy output
	output logic signed [31:0] vx_bounded, vy_bounded
);

	// ADJUST THIS PARAMETER, IT IS BEHAVING POORLY
	// This might not be the only issue but it's worth exploring
	localparam signed [31:0] turnfactor  	= 32'h00003999;
			
	localparam signed [31:0] avoidfactor 	= 32'h00000666;
	
	localparam signed [31:0] matchfactor 	= 32'h00000666;
	
	localparam signed [31:0] centerfactor 	= 32'h00000010;
	
	logic signed [31:0] div_val;
	
	// Instantiate lookup table for pre-calculated fractional dividers between 1 and 31
	// or multiply all of these things by zero if boid_ctr remains 0
	
	lut_32_divider lut(
		.lut_sel(boid_ctr_wb),
		.div_val(div_val)
	);
	
	logic signed [31:0] x_avg_n, y_avg_n, vx_avg_n, vy_avg_n;
	
	// At this point, if area is a concern and we want to pay more cycles to calculate all of this stuff on signoff,
	// we can reuse the same multipliers and add a register to bring our values through the same multipliers 3 times
	// This reduces the multipliers we need from 10 to 4. We can reduce the required multipliers to 2 if we spend
	// 6 cycles calculating, and we can reduce it to 1 if we spend 12 cycles calculating. This would also allow for
	// a reduction in the critical path, which very likely will exist in this module (representing the path from a
	// register through 3 multipliers, 5 adders, some control logic to adjudicate 2 of those additions, and the 
	// path to write this information back to the memory system.
	
	// Initial set of divisions (multiplication by 1/neighboring boids, or 0)
	
	fix15_mul f15_1(
		.a(x_avg_wb),
		.b(div_val),
		.q(x_avg_n)
	);
	
	fix15_mul f15_2(
		.a(y_avg_wb),
		.b(div_val),
		.q(y_avg_n)
	);
	
	fix15_mul f15_3(
		.a(vx_avg_wb),
		.b(div_val),
		.q(vx_avg_n)
	);
	
	fix15_mul f15_4(
		.a(vy_avg_wb),
		.b(div_val),
		.q(vy_avg_n)
	);
	
	// Factorization
	// Take the difference of the divided average and pos/vel and
	// normalize by the center/match factors
	
	logic signed [31:0] x_avg_f, y_avg_f, vx_avg_f, vy_avg_f;
	
	fix15_mul f15_11(
		.a(x_avg_n - x_wb),
		.b(SW[1] ? centerfactor : 0),
		.q(x_avg_f)
	);
	
	fix15_mul f15_21(
		.a(y_avg_n - y_wb),
		.b(SW[1] ? centerfactor : 0),
		.q(y_avg_f)
	);
	
	fix15_mul f15_31(
		.a(vx_avg_n - vx_wb),
		.b(SW[0] ? matchfactor : 0),
		.q(vx_avg_f)
	);
	
	fix15_mul f15_41(
		.a(vy_avg_n - vy_wb),
		.b(SW[0] ? matchfactor : 0),
		.q(vy_avg_f)
	);
	
	logic signed [31:0] vx_t_1, vy_t_1;
	
	assign vx_t_1 = x_avg_f + vx_avg_f;
	assign vy_t_1 = y_avg_f + vy_avg_f;
	
	// Factorize avoidance and sum into velocity
	
	logic signed [31:0] x_close_f, y_close_f;
	
	fix15_mul f15_12(
		.a(x_close_wb),
		.b(avoidfactor),
		.q(x_close_f)
	);
	
	fix15_mul f15_22(
		.a(y_close_wb),
		.b(avoidfactor),
		.q(y_close_f)
	);
	
	logic signed [31:0] vx_t_2, vy_t_2;
	
	logic signed [31:0] vx_t, vy_t;
	
	assign vx_t_2 = vx_wb + x_close_f;
	
	assign vy_t_2 = vy_wb + y_close_f;
	
	assign vx_t = vx_t_1 + vx_t_2;
	
	assign vy_t = vy_t_1 + vy_t_2;
	
	// Perform movement to drive boids away from boundaries
	
	logic [1:0] x_bchk, y_bchk;
	
	logic signed [31:0] x_max_b_t, y_max_b_t;
	
	assign x_max_b_t = $signed((32'd640 << 16) - x_bound);
	assign y_max_b_t = $signed((32'd480 << 16) - y_bound);
	
	assign x_bchk = {$signed (x_wb) > $signed(x_max_b_t), $signed(x_wb) < $signed(x_bound)};
	assign y_bchk = {$signed (y_wb) > $signed(y_max_b_t), $signed(y_wb) < $signed(y_bound)};
	
	always_comb begin
		case (x_bchk)
			2'd0: begin
					vx_bounded = vx_t;
				end
			2'd1: begin
					vx_bounded = vx_t + turnfactor;
				end
			2'd2: begin
					vx_bounded = vx_t - turnfactor;
				end
			default: begin
					vx_bounded = vx_t;
					// 3 is unreachable, contradictory state
				end 
		endcase
		
		case (y_bchk)
			2'd0: begin
					vy_bounded = vy_t;
				end
			2'd1: begin
					vy_bounded = vy_t + turnfactor;
				end
			2'd2: begin
					vy_bounded = vy_t - turnfactor;
				end
			default: begin
					vy_bounded = vy_t;
					// 3 is unreachable, contradictory state
				end 
		endcase
	end

endmodule