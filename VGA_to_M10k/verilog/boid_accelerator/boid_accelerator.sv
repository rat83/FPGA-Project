module boid_accelerator
#(
	parameter num_boids = 2
)
(
	input logic clk,
	input logic en,
	input logic reset,
	
	/*
	input logic [31:0]  
		x_in_b,  y_in_b, 	// current position
		px_in_b, py_in_b, 	// previous position (for erasure)
		vx_in_b, vy_in_b, 	// velocity
	*/
	
	
	
	input logic [31:0]  
		x,  y, 	// current position
	
	output logic is_boid_here
);

logic [31:0] 	x_m_d, y_m_d, vx_m_d, vy_m_d,
					x_d_m, y_d_m, vx_d_m, vy_d_m;

logic r_en_itr, r_en_tot;

logic [6:0] wb_en;

logic [$clog2(num_boids) - 1:0] 	which_boid;

xcel_dp the_dp 
(
	.clk(clk),
	.reset(reset),
	
	
	.r_en_itr(r_en_itr),
	.r_en_tot(r_en_tot),
	.wb_en(wb_en),
	
	.x_in_xcel(x_m_d),
	.y_in_xcel(y_m_d),
	.vx_in_xcel(vx_m_d),
	.vy_in_xcel(vy_m_d),
	
	.x_out_xcel(x_d_m),
	.y_out_xcel(y_d_m),
	.vx_out_xcel(vx_d_m),
	.vy_out_xcel(vy_d_m)
);

xcel_ctrl #(num_boids) the_ctrl 
(
	.clk(clk),
	.en(en),
	.reset(reset),
	
	.which_boid(which_boid),
	.r_en_tot(r_en_tot),
	.r_en_itr(r_en_itr),
	.wb_en(wb_en)
);

register_test_mem_wrapper #(num_boids) the_mem 
(
	.clk(clk),
	.reset(reset),
	.which_boid(which_boid),
	.wb_en(wb_en),
	
	.x_in_32(x_d_m),
	.y_in_32(y_d_m),
	.vx_in_32(vx_d_m),
	.vy_in_32(vy_d_m),
	
	.x_out_32(x_m_d),
	.y_out_32(y_m_d),
	.vx_out_32(vx_m_d),
	.vy_out_32(vy_m_d),
	
	.x_chk_in(x),
	.y_chk_in(y),
	.is_boid_here(is_boid_here)
);

endmodule



