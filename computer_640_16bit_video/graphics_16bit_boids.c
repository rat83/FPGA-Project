///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc graphics_video_16bit.c -o gr -O2 -lm
///
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h> 
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h>
#include <stdbool.h>
//#include "address_map_arm_brl4.h"

// video display
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define SDRAM_SPAN			  0x04000000
// characters
#define FPGA_CHAR_BASE        0xC9000000 
#define FPGA_CHAR_END         0xC9001FFF
#define FPGA_CHAR_SPAN        0x00002000
/* Cyclone V FPGA devices */
#define HW_REGS_BASE          0xff200000
//#define HW_REGS_SPAN        0x00200000 
#define HW_REGS_SPAN          0x00005000 

// graphics primitives
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_rect (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_Vline(int, int, int, short) ;
void VGA_Hline(int, int, int, short) ;
void VGA_disc (int, int, int, short);
void VGA_circle (int, int, int, int);

// 16-bit primary colors
#define red  (0+(0<<5)+(31<<11))
#define dark_red (0+(0<<5)+(15<<11))
#define green (0+(63<<5)+(0<<11))
#define dark_green (0+(31<<5)+(0<<11))
#define blue (31+(0<<5)+(0<<11))
#define dark_blue (15+(0<<5)+(0<<11))
#define yellow (0+(63<<5)+(31<<11))
#define cyan (31+(63<<5)+(0<<11))
#define magenta (31+(0<<5)+(31<<11))
#define black (0x0000)
#define gray (15+(31<<5)+(51<<11))
#define white (0xffff)
int colors[] = {red, dark_red, green, dark_green, blue, dark_blue, 
		yellow, cyan, magenta, gray, black, white};

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	int  *pixel_ptr ;\
	pixel_ptr = (int*)((char *)vga_pixel_ptr + (((y)*640+(x))<<1)) ; \
	*(short *)pixel_ptr = (color);\
} while(0)

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

// measure time
struct timeval t1, t2;
double elapsedTime;

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(( (((signed long long)(a)) << 15) / ((signed long long)(b))))

// number of Boids to spawn
#define BOID_COUNT 250

// Define boundary positions and velocity values
int boundary_bottom = 380;
int boundary_top = 100;
int boundary_left = 100;
int boundary_right = 540;
int max_v = 6;
int min_v = 3;

// Define parameters for Boids Algorithm
fix15 bias1Val = float2fix15(.001);
fix15 bias2Val = float2fix15(.001);

fix15 turnFactor = float2fix15(.2);
fix15 visualRange = int2fix15(40);
fix15 protectedRange = int2fix15(8);
fix15 centeringFactor = float2fix15(.0005);
fix15 avoidFactor = float2fix15(.05);
fix15 matchingFactor = float2fix15(.05);
fix15 maxspeed = int2fix15(6);
fix15 minspeed = int2fix15(3);
fix15 neighboring_boids = float2fix15(0);
fix15 protected_range_squared = float2fix15(64);
fix15 visual_range_squared = float2fix15(1600);

// Wall detection
#define hitBottom(b) (b>int2fix15(boundary_bottom))
#define hitTop(b) (b<int2fix15(boundary_top))
#define hitLeft(a) (a<int2fix15(boundary_left))
#define hitRight(a) (a>int2fix15(boundary_right))

// uS per frame
#define FRAME_RATE 33000

// State of boundary (0 = standard box, 1 = top-bottom wrapping, 2 = full wrapping)
int BoundaryState = 0;
int PreviousBoundaryState = 0; 
int ConfigurableWidth = 400;
int PreviousConfigurableWidth = 400;

int NumBiasedBoids1 = 0;
int NumBiasedBoids2 = 0;

int BoidOrigCount = 0;
int BoidDiff = 0;

bool whichBoid = 0;

// Global accumulator variables (zeroed at each Boid update)
// X and Y velocity to avoid other Boids
fix15 close_dx;
fix15 close_dy;

// X and Y average velocity of surrounding Boids
fix15 yvel_avg;
fix15 xvel_avg;

// X and Y average position of surrounding Boids
fix15 xpos_avg;
fix15 ypos_avg;

// Count of neighboring Boids 
fix15 count;

typedef struct{
  // current X and Y position of Boid
  fix15 x;
  fix15 y;

  // current X and Y velocity of Boid
  fix15 vx;
  fix15 vy;

  bool bias1;
  bool bias2;

} Boid;

Boid* boid_array [ BOID_COUNT*2 ];

int rrand(int low, int high){
  return (int)((rand() % (high - low + 1)) + low);
}

int coinrand(){
  return (int)((rand() * 2) / 2);
}

// Create a boid
Boid* boid_constructor()
{
  Boid* new_boid = malloc(sizeof(Boid));

  // Start in at random value
  new_boid->x = int2fix15(rrand(boundary_left, boundary_right)) ;
  new_boid->y = int2fix15(rrand(boundary_top, boundary_bottom)) ;
  new_boid->bias1 = 0;
  new_boid->bias2 = 0;
  // Choose left or right
  int direction_x = coinrand();
  if (direction_x == 1) new_boid->vx = int2fix15(rrand(min_v, max_v)) ;
  else new_boid->vx = int2fix15(rrand(-1 * max_v, -1 * min_v)) ;
  // Chose down or up
  int direction_y = coinrand();
  if (direction_y == 1) new_boid->vy = int2fix15(rrand(min_v, max_v)) ;
  else new_boid->vy = int2fix15(rrand(-1 * max_v, -1 * min_v)) ;
  printf("Spawned boid! POSX %d POSY %d\n", fix2int15(new_boid->x),fix2int15(new_boid->y));
  return new_boid;
}

fix15 alpha_max_beta_min(fix15 a, fix15 b){
fix15 max = (a > b) ? a : b;
fix15 min = (a > b) ? b : a;

return(max + (min >> 1));
}

void screenEnds(Boid* boid)
{
  if (BoundaryState == 1) { // Top_bottom wrapping
    if (hitLeft(boid->x)) boid->vx = boid->vx + turnFactor;
    else if (hitRight(boid->x)) boid->vx = boid->vx - turnFactor;

    if (hitBottom(boid->y)) boid->y = boid->y - int2fix15(boundary_bottom); 
    else if (hitTop(boid->y)) boid->y = boid->y + int2fix15(boundary_bottom);
  }
  else if (BoundaryState == 2) { // Total wrapping
    if (hitLeft(boid->x)) boid->x = boid->x + int2fix15(boundary_right);
    else if (hitRight(boid->x)) boid->x = boid->x - int2fix15(boundary_right);

    if (hitBottom(boid->y)) boid->y = boid->y - int2fix15(boundary_bottom); 
    else if (hitTop(boid->y)) boid->y = boid->y + int2fix15(boundary_bottom);
  } 
  else { // Standard Arena
    if (hitLeft(boid->x)) boid->vx = boid->vx + turnFactor;
    else if (hitRight(boid->x)) boid->vx = boid->vx - turnFactor; 

    if (hitBottom(boid->y)) boid->vy = boid->vy - turnFactor; 
    else if (hitTop(boid->y)) boid->vy = boid->vy + turnFactor;
  }
}


void zeroAllAccum(){
  xpos_avg = 0;
  ypos_avg = 0;
  xvel_avg = 0;
  yvel_avg = 0;
  neighboring_boids = 0;
  close_dx = 0;
  close_dy = 0; 
}

void boidSeparation(Boid* boid, int idx){

  fix15 dx = boid->x - boid_array[idx]->x;
  fix15 dy = boid->y - boid_array[idx]->y; 

  if ( (abs(dx) < visualRange) && (abs(dy)< visualRange) ){
    fix15 squared_distance = multfix15(dx,dx) + multfix15(dy,dy);

    if (squared_distance < protected_range_squared){
      close_dx += dx;
      close_dy += dy;
    }
    else if(squared_distance < visual_range_squared){
      xpos_avg += boid_array[idx]->x;
      ypos_avg += boid_array[idx]->y;
      xvel_avg += boid_array[idx]->vx;
      yvel_avg += boid_array[idx]->vy;
      neighboring_boids = neighboring_boids + float2fix15(1);
    }
  }
}
  



void boidCalculation(Boid* boid){
  if(neighboring_boids > 0){
    xpos_avg = divfix(xpos_avg,neighboring_boids);
    ypos_avg = divfix(ypos_avg,neighboring_boids);
    xvel_avg = divfix(xvel_avg,neighboring_boids);
    yvel_avg = divfix(yvel_avg,neighboring_boids);

    boid->vx += multfix15((xvel_avg - boid->vx),matchingFactor);
    boid->vx += multfix15((xpos_avg - boid->x ),centeringFactor);

    boid->vy += multfix15((yvel_avg - boid->vy),matchingFactor);
    boid->vy += multfix15((ypos_avg - boid->y ),centeringFactor);
  }

  boid->vx += multfix15(close_dx,avoidFactor);
  boid->vy += multfix15(close_dy,avoidFactor);

  if(boid->bias1){
    boid->vx = multfix15((int2fix15(1) - bias1Val),boid->vx) + bias1Val;
  }

  if(boid->bias2){
    boid->vx = multfix15((int2fix15(1) - bias2Val),boid->vx) - bias2Val;
  }

  screenEnds(boid);

  fix15 a = abs(boid->vx);
  fix15 b = abs(boid->vy);

  fix15 max = (a > b) ? a : b;
  fix15 min = (a > b) ? b : a;

  fix15 speed = max + (min >> 1);

  if ( speed < minspeed ){
    boid->vx = multfix15(divfix(boid->vx,speed),minspeed);
    boid->vy = multfix15(divfix(boid->vy,speed),minspeed);
  }

  if ( speed > maxspeed ){
    boid->vx = multfix15(divfix(boid->vx,speed), maxspeed);
    boid->vy = multfix15(divfix(boid->vy,speed), maxspeed);
  }

  boid->x = boid->x + boid->vx ;
  boid->y = boid->y + boid->vy ;
}

void boidUpdate(Boid* boid, int idx){
  zeroAllAccum();

  if(BoidDiff > 0){
    if(!whichBoid){
      if(BoidOrigCount > NumBiasedBoids1){
        if(boid->bias1){
          boid->bias1 = 0;
          BoidDiff--;
        }
      }
      else if(BoidOrigCount < NumBiasedBoids1){
        if (!boid->bias1 && !boid->bias2){
          boid->bias1 = 1;
          BoidDiff--;
        }
      }
      else{
        BoidDiff = 0;
      }
    }
    else{ 
      if(BoidOrigCount > NumBiasedBoids2){
        if(boid->bias2){
          boid->bias2 = 0;
          BoidDiff--;
        }
      }
      else if(BoidOrigCount < NumBiasedBoids2){
        if (!boid->bias1 && !boid->bias2){
          boid->bias2 = 1;
          BoidDiff--;
        }
      }
      else{
        BoidDiff = 0;
      }
    }
  }

  for(int i =0; i < BOID_COUNT*2; i++){
    if(i != idx){
      boidSeparation(boid, i);
    }
  }
  boidCalculation(boid); 
}

// Draw the boundaries
void drawArena() {
  // Erase screen if change in Boundary State
  if (BoundaryState != PreviousBoundaryState || ConfigurableWidth != PreviousConfigurableWidth) {
    VGA_box(0, 0, 640, 480, black);
    PreviousBoundaryState = BoundaryState;
    PreviousConfigurableWidth = ConfigurableWidth;
  }

  // screen size 640x480
  if (BoundaryState == 0) { // Standard Arena
    VGA_Vline(100, 100, 280, white) ;
    VGA_Vline(540, 100, 280, white) ;
    VGA_Hline(100, 100, 440, white) ;
    VGA_Hline(100, 380, 440, white) ;
  } 
  else if (BoundaryState == 1) { // Top-Bottom Wrapping
    VGA_Vline(boundary_left, 0, 480, white) ;
    VGA_Vline(boundary_right, 0, 480, white) ;
  }
  else { // Full Wrapping
    // no arena
  }
}

	
int main(void)
{
  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}
    

	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, SDRAM_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
	char text_bottom_row[40] = "Cornell ece5760\0";
	char text_next[40] = "Graphics primitives\0";
	char num_string[20], time_string[20] ;
	char color_index = 0 ;
	int color_counter = 0 ;
	
	// position of disk primitive
	int disc_x = 0;
	// position of circle primitive
	int circle_x = 0 ;
	// position of box primitive
	int box_x = 5 ;
	// position of vertical line primitive
	int Vline_x = 350;
	// position of horizontal line primitive
	int Hline_y = 250;

	//VGA_text (34, 1, text_top_row);
	//VGA_text (34, 2, text_bottom_row);
	// clear the screen
	VGA_box (0, 0, 639, 479, 0x0000);
	// clear the text
	VGA_text_clear();
	// write text
	VGA_text (10, 1, text_top_row);
	VGA_text (10, 2, text_bottom_row);
	VGA_text (10, 3, text_next);
	
	// R bits 11-15 mask 0xf800
	// G bits 5-10  mask 0x07e0
	// B bits 0-4   mask 0x001f
	// so color = B+(G<<5)+(R<<11);
	
    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // Spawn a boid
    int itr = 0;
    for(int i = 1; i < BOID_COUNT*2; i=i+2){
      boid_array[i] = boid_constructor();
      if(itr < NumBiasedBoids2){
        boid_array[i]->bias2 = 1;
        itr++;
      }
    }


	while(1) 
	{
	  // Measure time at start of thread
      begin_time = time_us_32() ; 

      for (int i = 1; i < BOID_COUNT*2; i=i+2) 
      {
      // erase boid
      VGA_rect(fix2int15(boid_array[i]->x), fix2int15(boid_array[i]->y), 2, 2, black);
      // update boid's position and velocity
      boidUpdate(boid_array[i], i);
      // draw the boid at its new position
      if(boid_array[i]->bias1){
        VGA_rect(fix2int15(boid_array[i]->x), fix2int15(boid_array[i]->y), 2, 2, red); 
      }
      else if(boid_array[i]->bias2){ 
        VGA_rect(fix2int15(boid_array[i]->x), fix2int15(boid_array[i]->y), 2, 2, blue);
      }
      else{
        VGA_rect(fix2int15(boid_array[i]->x), fix2int15(boid_array[i]->y), 2, 2, white); 
      }
      // draw the boundaries
      }

    VGA_text_clear();
	// write text
	VGA_text (10, 1, text_top_row);
	VGA_text (10, 2, text_bottom_row);
	VGA_text (10, 3, text_next);
           
      drawArena() ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
	} // end while(1)
} // end main

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<79; x++){
		for (y=0; y<59; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			//*(char *)pixel_ptr = pixel_color;	
			VGA_PIXEL(col,row,pixel_color);	
		}
}

/****************************************************************************************
 * Draw a outline rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	// left edge
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
		
	// right edge
	col = x2;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
	
	// top edge
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);
	}
	
	// bottom edge
	row = y2;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);
	}
}

/****************************************************************************************
 * Draw a horixontal line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Hline(int x1, int y1, int x2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (x1>x2) SWAP(x1,x2);
	// line
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
}

/****************************************************************************************
 * Draw a vertical line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Vline(int x1, int y1, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (y2<0) y2 = 0;
	if (y1>y2) SWAP(y1,y2);
	// line
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);			
	}
}


/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++)
		for (xc = -r; xc <= r; xc++)
		{
			col = xc;
			row = yc;
			// add the r to make the edge smoother
			if(col*col+row*row <= rsqr+r){
				col += x; // add the center point
				row += y; // add the center point
				//check for valid 640x480
				if (col>639) col = 639;
				if (row>479) row = 479;
				if (col<0) col = 0;
				if (row<0) row = 0;
				//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
				// set pixel color
				//*(char *)pixel_ptr = pixel_color;
				VGA_PIXEL(col,row,pixel_color);	
			}
					
		}
}

/****************************************************************************************
 * Draw a  circle on the VGA monitor 
****************************************************************************************/

void VGA_circle(int x, int y, int r, int pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	int col1, row1;
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++){
		//row = yc;
		col1 = (int)sqrt((float)(rsqr + r - yc*yc));
		// right edge
		col = col1 + x; // add the center point
		row = yc + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		col = -col1 + x; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
	for (xc = -r; xc <= r; xc++){
		//row = yc;
		row1 = (int)sqrt((float)(rsqr + r - xc*xc));
		// right edge
		col = xc + x; // add the center point
		row = row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		row = -row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	char *pixel_ptr ;
	
	/* check and fix line coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		//*(char *)pixel_ptr = c;
		VGA_PIXEL(x,y,c);			
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}
