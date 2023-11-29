module register_test_memory
#(
	parameter num_boids = 2
)
(	
	input logic clk,
	input logic reset,
	input logic	[$clog2(num_boids)-1:0] which_boid,
	
	input	logic	[6:0] 	wb_en,
	
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
	output logic	[31:0]	vy_acc_out,
	
	input logic 	[31:0]	x_chk_in,
	input logic 	[31:0]	y_chk_in,
	output logic				is_boid_here
	
	// To investigate: making this a bi-directional bus
	
);
	logic [27:0] x_t 			[num_boids-1:0];
	logic [26:0] y_t 			[num_boids-1:0];
	logic [20:0] vx_t 		[num_boids-1:0];
	logic [20:0] vy_t 		[num_boids-1:0];
	logic [31:0] vx_acc_t 	[num_boids-1:0]; 
	logic [31:0] vy_acc_t 	[num_boids-1:0];
													
	// input genvar block, will instantiate test memory
						
	
	generate 
		genvar i;
		for (i = 0; i < num_boids; i++) begin : tmem
		// each reg is muxed to state:
		
		d_reg #(28, ((28'd110 + 28'd5 * (i % 64)) << 16)) x
		(	
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && wb_en[1] && wb_en[0]) ? x_in : x_t[i]),
			.q(x_t[i])
		);
		
		d_reg #(27, ((28'd110 + 28'd5 * (i / 512)) << 16)) y
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && wb_en[2] && wb_en[0]) ? y_in : y_t[i]),
			.q(y_t[i])
		);
		
		d_reg #(21, ((28'd5) << 16)) vx
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && wb_en[3] && wb_en[0]) ? vx_in : vx_t[i]),
			.q(vx_t[i])
		);
		
		d_reg #(21, ((28'd4) << 16)) vy
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && wb_en[4] && wb_en[0]) ? vy_in : vy_t[i]),
			.q(vy_t[i])
		);
		
		d_reg #(32, (0)) x_a
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && wb_en[5] && wb_en[0]) ? vx_acc_in : vx_acc_t[i]),
			.q(vx_acc_t[i])
		);
		
		d_reg #(32, (0)) y_a
		(
			.clk(clk),
			.reset(reset),
			.d((which_boid == i && wb_en[6] && wb_en[0]) ? vy_acc_in : vy_acc_t[i]),
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
	
	logic [num_boids-1:0] is_boid_here_t;
	
	logic [31:0] boid_comp_x [num_boids-1:0];
	
	logic [31:0] boid_comp_y [num_boids-1:0];
	
	integer j;
	always @(*) begin
		for(j = 0; j < num_boids; j++) begin
			boid_comp_x[j] = {{27{x_t[j][27]}}, (x_t[j] >>> 16)};
			boid_comp_y[j] = {{26{y_t[j][26]}}, (y_t[j] >>> 16)};
			is_boid_here_t[j] = ((boid_comp_x[j] == x_chk_in) && (boid_comp_y[j] == y_chk_in));
		end
	end
	
	always @(*) begin
	/*
		is_boid_here = '0; // fill 0
		foreach(is_boid_here_t[k]) begin
			is_boid_here |= is_boid_here_t[k];
		end
		*/
		
		is_boid_here = |is_boid_here_t;
	end
	
endmodule

module register_test_mem_wrapper
#(
	parameter num_boids = 2
)
(	
	input logic clk,
	input logic reset,
	input logic	[$clog2(num_boids)-1:0] which_boid,
	output logic is_refilling,
	
	input	logic	[6:0] 	wb_en,
	
	// input ports
	
	input logic	[31:0] 	x_in_32,
	input logic	[31:0]	y_in_32,
	input	logic	[31:0]	vx_in_32,
	input	logic	[31:0] 	vy_in_32,
	
	input	logic	[31:0]	vx_acc_in,
	input logic	[31:0]	vy_acc_in,
	
	// output ports
	
	output logic	[31:0] 	x_out_32,
	output logic	[31:0]	y_out_32,
	output logic	[31:0]	vx_out_32,
	output logic	[31:0] 	vy_out_32,
	
	output logic	[31:0]	vx_acc_out,
	output logic	[31:0]	vy_acc_out,
	
	input logic 	[31:0]	x_chk_in,
	input logic 	[31:0]	y_chk_in,
	output logic				is_boid_here
	
	
	// To investigate: making this a bi-directional bus
	
);
	
	assign is_refilling = 1'b0;
	
	// This module truncates incoming values to the memory system 
	// and pads outgoing values from the memory system
	
	// Memory bit-width values
	logic	[27:0] 	x_in;
	logic	[26:0]	y_in;
	logic	[20:0]	vx_in;
	logic	[20:0] 	vy_in;
	
	logic	[27:0] 	x_out;
	logic	[26:0]	y_out;
	logic	[20:0]	vx_out;
	logic	[20:0] 	vy_out;
	
	assign x_in = x_in_32[27:0];
	assign y_in = y_in_32[26:0];
	assign vx_in = vx_in_32[20:0];
	assign vy_in = vy_in_32[20:0];
	
	// This should be able to be replaced by a M10k memory
	register_test_memory #(num_boids) rtm(.*);
	
	zero_pad_fix15
	#(28,32)
	x_out_pad
	(
		.fix_in(x_out),
		.fix_out(x_out_32)
	);
	
	zero_pad_fix15
	#(27,32)
	y_out_pad
	(
		.fix_in(y_out),
		.fix_out(y_out_32)
	);
	
	zero_pad_fix15
	#(21,32)
	vx_out_pad
	(
		.fix_in(vx_out),
		.fix_out(vx_out_32)
	);
	
	zero_pad_fix15
	#(21,32)
	vy_out_pad
	(
		.fix_in(vy_out),
		.fix_out(vy_out_32)
	);

endmodule

module mem_initializer
#(
	parameter num_boids = 2
)
(
	input 	logic clk,
	input 	logic reset,
	output 	logic is_refilling,
	
	output	logic	[6:0] 	wb_en,
	output	logic	[$clog2(num_boids)-1:0] which_boid,
	
	output logic	[27:0] 	x_out_r,
	output logic	[26:0]	y_out_r,
	output logic	[20:0]	vx_out_r,
	output logic	[20:0] 	vy_out_r
);

logic [$clog2(num_boids)-1:0] up_ctr;

logic [1:0] state, next_state;

always @(posedge clk) begin
	if (reset) begin
		state <= 2'b0;
	end else begin
		state <= next_state;
	end
	
	if (state == 1) begin
		up_ctr <= up_ctr + 1;
	end else if (state == 0) begin
		up_ctr <= 0;
	end
end

always @(*) begin
	case(state)
		0: begin next_state = 1; end
		1:	begin next_state = 2; end
		2:	begin next_state = (up_ctr >= num_boids) ? 3 : 1;end
		3:	begin next_state = 3;end
		default: begin end
	endcase
end

always @(*) begin
	is_refilling = (state != 2'd3);
	wb_en = 7'b0;
	x_out_r = 0;
	y_out_r = 0;
	vx_out_r = 0;
	vy_out_r = 0 ;
	case(state)
		0: begin end
		1: begin end
		2: begin
			
			wb_en = 7'b0011111;
			x_out_r = ((28'd100 + 28'd10 * (up_ctr % 540)) << 16);
			y_out_r = ((28'd100 + 28'd10 * (up_ctr / 280)) << 16);
			vx_out_r = ((21'd4) << 16); 
			vy_out_r = ((21'd4) << 16); 
		end
		3: begin end
		default: begin end
	endcase
end

endmodule
