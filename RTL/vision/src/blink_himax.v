/****************************************************************************
 Copyright (c) 2021 tinyVision.ai Inc.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 4. The Software/Firmware is used solely in conjunction with devices provided by
 tinyVision.ai Inc. 

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 For a license to use on non-tinyVision.ai Inc. hardware, please contact license@tinyvision.ai
 */

/**
 * Module:: Top level jumping off point to get pixel data from the camera
 */
// `define USE_RGB
module blink_himax (
    // Host interface
    input         uart_rxd  /* synthesis syn_keep = 1 */ ,
    output        uart_txd  /* synthesis syn_noprune = 1 */ ,
    // GPIO
    output  [2:0] oled      /* synthesis syn_noprune = 1 */,
    output        enable   /* synthesis syn_noprune = 1 */,
    // SPI master to control imager and IMU
    output         cam_scl   ,
    inout          cam_sda   /* synthesis syn_keep = 1 */,
    output        cam_mclk , // Must be ~6MHz
    // Image sensor data port
    input         cam_pclk  /* synthesis syn_keep = 1 */ /* synthesis syn_noprune = 1 */ ,
    input         cam_vsync  /* synthesis syn_keep = 1 */ /* synthesis syn_noprune = 1 */  ,
    input         cam_hsync  /* synthesis syn_keep = 1 */ /* synthesis syn_noprune = 1 */  ,
    // Himax is setup for 4 bit output. Note that the bits are reversed for Himax to help with routing!
    input   [7:0] cam_datap   /* synthesis syn_keep = 1 */    ,
//	input   [7:4] cam_datan	,
    output        cam_trig   , // sensor_led,
    output        rgb_red   /* synthesis syn_noprune = 1 */ ,
    output        rgb_green /* synthesis syn_noprune = 1 */,
    output        rgb_blue  /* synthesis syn_noprune = 1 */,
	output          miso

);
    //Parameters

    wire clk  ;
    wire rst_n, rst;
	
wire		w_uart_txd;
wire		w_uart_rxd;
wire			w_frame_req ;
wire	   w_uart_empty;
wire	   w_cam_pclk /* synthesis syn_keep = 1 */;	
wire	   w_clk;
wire       w_cam_vsync /* synthesis syn_keep = 1 */ ;
wire 	   w_cam_hsync /* synthesis syn_keep = 1 */ ;

wire	   w_cam_scl_in;
wire	   w_cam_sda_in;

	assign w_cam_scl_in = cam_scl;
	assign w_cam_sda_in = cam_sda;
	
	assign enable =  1'b1; // power on LDO's on camera board
	assign w_cam_vsync = cam_vsync;
	assign w_cam_hsync = cam_hsync;
	
    // Internal oscillator
    HSOSC #(.CLKHF_DIV("0b01")) u_hfosc (
        .CLKHFEN(1'b1),
        .CLKHFPU(1'b1),
        .CLKHF  (clk )
    );

	assign w_cam_pclk = cam_pclk;
	assign w_clk = clk;
	assign clk_24m = clk;

    ice40_resetn u_reset_n (.clk(clk), .resetn(rst_n));
    assign rst = ~rst_n;
`ifdef USE_RGB
    logic [27:0] clk_divider;
    always @(posedge clk) begin
        if (rst)
            clk_divider <= 0;
        else
            clk_divider <= clk_divider + 1;
    end
`endif
`ifdef NOT_DEFINED
    // Program the Himax on boot up

    // Simple state machine to deal with initialization etc.
    reg init, init_done;

    enum {S_INIT_WAIT, S_PROG, S_WAIT, S_DONE, S_XXX} s_state, s_next;
    reg [20:0] timer;
    always_ff @(posedge clk) begin
        if(rst) s_state <= S_INIT_WAIT;
        else s_state <= s_next;
    end

    always_comb begin
        s_next = S_XXX;
        case (s_state)
            S_INIT_WAIT : if (timer == '1) s_next = S_PROG; else s_next = S_INIT_WAIT;
            S_PROG      : s_next = S_WAIT;
            S_WAIT      : if (init_done) s_next = S_DONE; else s_next = S_WAIT;
            S_DONE      : s_next = S_DONE;
            S_XXX       : s_next = S_XXX;
        endcase // s_state
    end

    always_ff @(posedge clk) begin
        if(rst) begin
            timer <= 0;
            init  <= 0;
        end else begin
            init <= '0;

            case (s_state)
                S_INIT_WAIT : timer <= timer + 1;
                S_PROG      : init <= 1;
                S_WAIT      : timer <= 0;
                S_DONE      : ;
                S_XXX       : ;
            endcase // s_state
        end
    end
`endif

    // Program the Himax on boot up

    // Simple state machine to deal with initialization etc.
    reg init;
	wire init_done, w_init;
	
	assign w_init = init;
	
parameter [2:0]
          S_INIT_WAIT = 3'b000,
		  S_PROG	  = 3'b001,
		  S_WAIT	  = 3'b010,
		  S_DONE	  = 3'b011,
		  S_XXX		  = 3'b111;	

//    enum {S_INIT_WAIT, S_PROG, S_WAIT, S_DONE, S_XXX} s_state, s_next;
    reg s_state, s_next;
    reg [20:0] timer;
    always @(posedge clk_24m) begin
        if(rst) s_state <= S_INIT_WAIT;
        else s_state <= s_next;
    end

    always @(posedge clk_24m) 
		begin
        s_next = S_XXX;
		
        case (s_state)
            S_INIT_WAIT : if (timer == "21'b000000000000000000001") s_next = S_PROG; else s_next = S_INIT_WAIT;
            S_PROG      : s_next = S_WAIT;
            S_WAIT      : if (init_done) s_next = S_DONE; else s_next = S_WAIT;
            S_DONE      : s_next = S_DONE;
            S_XXX       : s_next = S_XXX;
			 default :
                    s_next <= S_INIT_WAIT;
        endcase // s_state
    end

    always @(posedge clk_24m) begin
        if(rst) begin
            timer <= 0;
            init  <= 0;
        end else begin
            init <= 0;

            case (s_state)
                S_INIT_WAIT : timer <= timer + 1;
                S_PROG      : init <= 1;
                S_WAIT      : timer <= 0;
                S_DONE      : ;
                S_XXX       : ;
				default :
                    s_next <= S_INIT_WAIT;
            endcase // s_state
        end
    end

    wire w_scl_out, w_sda_out;
    lsc_i2cm_himax #(.EN_ALT(0), .INIT_FILE("ram256x16_himax_324x324_1fps.mem")) u_lsc_i2cm_himax (
        .clk      (w_clk      ),
        .init     (w_init     ),
        .init_done(init_done),
        .scl_in   (w_cam_scl_in  ),
        .sda_in   (w_cam_sda_in  ),
        .scl_out  (w_scl_out),
        .sda_out  (w_sda_out),
        .resetn   (rst_n    )
    );

    assign cam_scl = w_scl_out ? 1'bz : 1'b0;
    assign cam_sda = w_sda_out ? 1'bz : 1'b0;

    // Sensor uses its internal clock except during configuration
    assign cam_mclk = init_done ? 1'b0 : clk;

    // @TBD!
//    assign sensor_led = '1;
	assign cam_trig = 1;
    //================================================================================
    // Capture data and convert to 8 bits wide
    //================================================================================
    reg [7:0] cam_data;
//    reg [3:0] pxd_d   ;
    reg       hsync, vsync /* synthesis syn_keep = 1 */;
    reg [9:0] pxl_cnt ;
    wire       pxl_vld ;
	
	assign miso = vsync;

    always @(posedge w_cam_pclk) begin
 //       pxd_d    <= cam_datap;
        cam_data <= cam_datap;
        hsync    <= w_cam_hsync;
        vsync    <= w_cam_vsync;
		
        if (w_cam_hsync == 0)
            pxl_cnt <= 0;
        else
            pxl_cnt <= pxl_cnt + 1;
    end
	
    assign pxl_vld = hsync && ~pxl_cnt[0];
/*	
   reg [7:0] pixel_data;
   reg       pixel_vld ;
   wire      w_wfull, w_awfull, w_rempty, w_arempty;
   
	async_fifo #(.DSIZE(8),.ASIZE(4),.FALLTHROUGH("TRUE") ) i_async_fifo (   // First word fall-through
    .wclk (w_cam_pclk    ),
    .wrst_n (rst_n       ),
    .winc (1'b1),
    .wdata (cam_data  ),
    .wfull (w_wfull),
    .awfull (w_awfull),
    .rclk (w_clk       ),
    .rrst_n (rst_n       ),
    .rinc (1'b1),
    .rdata (pixel_data),
    .rempty (w_rempty),
    .arempty (w_arempty)
    );	
*/	
    // Cross the clock domain into the local clock	
    wire [7:0] pixel_data;
    wire       pixel_vld ;
    cc561 #(.DW(8)) i_cc561 (
        .aclk (w_cam_pclk    ),
        .arst (rst       ),
        .adata(cam_datap  ),
        .aen  (pxl_vld   ),
        .bclk (clk       ),
        .bdata(pixel_data),
        .ben  (pixel_vld )
    );
	
//	assign pixel_vld = pxl_vld;

    lsc_uart #(.PERIOD(16'd103), .BUFFER_SIZE("4K")) u_lsc_uart(		// ~230.4kB at 24MHz clock
	.ref_clk(w_clk         ),
	.clk    (w_clk         ),
//	.i_din  (cam_datap),	
	.i_din  (pixel_data),
//	.i_din  (cam_datap),
//	.i_valid(pxl_vld), 
	.i_valid(pixel_vld), 
//	.i_valid(pxl_vld), 
	.o_dout (),
	.o_valid(w_frame_req ),
	.o_empty(),
	.i_rxd  (w_uart_rxd  ), 
	.o_txd  (w_uart_txd  ),
	.resetn (rst_n      )
    );


//	wire uRdy;
/* lightweight uartTx for debug ---  */
/*	UartTx #( .BAUD_PERIOD(16'd103) ) u_uart_tx (		// 207 @ 24 MHz
	.clk(w_clk),
	.rst(rst),
//	.go(1'b1),
	.go(pixel_vld),	
	.data(pixel_data),
	.rdy(uRdy),
	.tx(w_uart_txd)
	);
*/
	assign uart_txd =  w_uart_txd; // power on for UPDUINO2	
	assign w_uart_rxd = uart_rxd;
    assign w_uart_empty = 1'b1;
	
`ifdef USE_RGB
    //================================================================================
    // LED ports have to have a special IO driver
    //================================================================================
    // LED is too bright, make this dim enough to not hurt eyes!
    logic duty_cycle;
    assign duty_cycle = &clk_divider[6:0];
//	assign duty_cycle = clk_divider[6:0];
    logic red, green, blue;
	
RGB
#(
  .CURRENT_MODE ("1"),
  .RGB0_CURRENT ("0b000001"),
  .RGB1_CURRENT ("0b000001"),
  .RGB2_CURRENT ("0b000001")
) u_led_driver (
  .CURREN   (1'b1),  // I
  .RGBLEDEN (1'b1),  // I
  .RGB0PWM  (red),  // I
  .RGB1PWM  (green),  // I
  .RGB2PWM  (blue),  // I
  .RGB2     (rgb_red),  // O
  .RGB1     (rgb_blue),  // O
  .RGB0     (rgb_green)   // O
);	

/*
    RGB u_led_driver (
        .CURREN  (1'b1     ),
        .RGBLEDEN(1'b1     ),
        .RGB0PWM (red      ),
        .RGB1PWM (green    ),
        .RGB2PWM (blue     ),
        .RGB0    (rgb_red  ),
        .RGB1    (rgb_green),
        .RGB2    (rgb_blue )
    );
    defparam u_led_driver.CURRENT_MODE = "1" ;
    defparam u_led_driver.RGB0_CURRENT = "0b000001";
    defparam u_led_driver.RGB1_CURRENT = "0b000001";
    defparam u_led_driver.RGB2_CURRENT = "0b000001";
*/
    assign green = cam_vsync && duty_cycle;
	

    assign enable = pixel_vld;
    assign oled[0]   = pixel_data[0];
    assign oled[1]   = pixel_data[2];
    assign oled[2]   = pixel_data[7];
`endif

reg green, blue, red;

assign rgb_green = green;
assign rgb_blue = blue;
assign rgb_red = red;
assign oled[0] = green;
assign oled[1] = blue;

reg [27:0] counter;
// double precision counter1;
reg state;
reg cam_trig_reg;

//	assign cam_trig = cam_trig_reg;

always @ (posedge clk) begin
	
	if (rst)
		counter <= 0;
		else
    counter <= counter + 1;
		 
	case (counter)
		
		 10000000: 
		 begin
		 state <= 1'b0;  // 
		 green <= state;
		 cam_trig_reg <= 1'b0;
		 end
		 20000000:
		 begin 
		 state <= 1'b1;
		 green <= state;
		 cam_trig_reg <= 1'b1;
		 end
		 35000000: 
		 begin 
         state <= 1'b0;  // 
		 blue <= state;
		 cam_trig_reg <= 1'b0;
		 end
		 45000000: 
		 begin
		 state <= 1'b1;
		 blue <= state;
		 cam_trig_reg <= 1'b1;
		 end
		 65000000: 
		 begin
		 state <= 1'b0;
		 red <= state;
		 cam_trig_reg <= 1'b0;
		 end
		 85000000: 
		 begin
		 state <= 1'b1; // 
		 red <= state;
		 cam_trig_reg <= 1'b1;
		 end
		 95000000: 
		 begin 
		 state <= 1'b0;
		 green <= state;
		 cam_trig_reg <= 1'b0;
		 end
`ifdef TEST_BUILD		 
		115000000: 
		begin
		state <= 1'b1;
		green <= state;
		end
		135000000: 
		begin
		state <= 1'b0;  // 
		blue <= state;
		end
		145000000: 
		begin
		state <= 1'b1;
		blue <= state;
		end
		155000000: 
		begin
		state <= 1'b0;
		red <= state;
		end
		165000000: 
		begin 
		state <= 1'b1;
		red <= state;
		end
		175000000: 
		begin
		state <= 1'b0;
		green <= state;
		end
		185000000: 
		begin
		state <= 1'b1;
		green <= state;
		end
		200000000: 
		begin 
		state <= 1'b0;   // 
		blue <= state;
		end
		210000000: 
		begin
		state <= 1'b1;
		blue <= state;
		end
		220000000: 
		begin
		state <= 1'b0;
		red <= state;
		end
		230000000: 
		begin
		state <= 1'b1;
		red <= state;
		end
		240000000: 
		begin
		state <= 1'b0;
		green <= state;
		end
		250000000: 
		begin
		state <= 1'b1;
		green <= state;
		end
		260000000: 
		begin
		state <= 1'b0;
		blue <= state;
		end
		270000000: 
		begin
		state <= 1'b1;
		blue <= state;
		end
		290000000: 
		begin
		state <= 1'b0;  // 
		red <= state;
		end
		300000000: 
		begin
		state <= 1'b1;
		red <= state;
		end
		310000000: 
		begin
		state <= 1'b0;
		green <= state;
		end
		320000000: 
		begin
		state <= 1'b1;
		green <= state;
		end
		340000000: 
		begin
		state <= 1'b0;	// 
		blue <= state;
		end
		360000000: 
		begin
		state <= 1'b1;
		blue <= state;
		end
		380000000: 
		begin
		state <= 1'b0;	// 
		red <= state;
		end
		400000000: 
		begin
		state <= 1'b1;
		red <= state;
		end		
`endif		
		default:   if (counter >= 95001000 )begin
			counter <=0;
//			cam_trig_reg <= 1'b1;
//		state <= !state;
	 end	
		
    endcase
		 if(counter >= 95001000 )begin		// 3600010000 95000000 400001000
		counter <= 0;
	end
end
endmodule




