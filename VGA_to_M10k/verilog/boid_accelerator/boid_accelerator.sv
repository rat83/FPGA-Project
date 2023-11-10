module boid_accelerator(
	input logic clk,
	input logic en,
	input logic reset,
	
	/*
	input logic [31:0]  
		x_in_b,  y_in_b, 	// current position
		px_in_b, py_in_b, 	// previous position (for erasure)
		vx_in_b, vy_in_b, 	// velocity
	*/
	
	output logic [31:0]  
		x,  y 	// current position
	
);

endmodule



