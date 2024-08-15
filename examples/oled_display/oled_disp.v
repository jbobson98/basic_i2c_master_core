module oled_disp (
    input wire  i_clk,
    input wire  i_rst,
    input wire  i_cmd_1,
    input wire  i_cmd_2,
    output wire o_oled_rst,
    inout wire  i2c_sda,
    inout wire  i2c_scl
);

/* Top Level Parameters */
localparam CLK_FREQ         = 100_000_000;
localparam I2C_FAST_MODE_EN = 0;

/* Oled Display Commands */
localparam OLED_DEV_ADDR = 10'h3C;    // [Pin4->GND: addr = 0x3C] [Pin4->VCC: addr = 0x3D]

// o_oled_rst should be kept high during normal operation, pulling low resets the oled chip
// pull reset low for 10ms to do reset


/* Debounce and synchronize buttons */
wire rst_sync, cmd_1_sync, cmd_2_sync;
debouncer #(.CLK_FREQ(CLK_FREQ)) reset_debouncer 
    (.clk(i_clk), .rst(1'b0), .btn_in(i_rst), .btn_out(rst_sync));
debouncer #(.CLK_FREQ(CLK_FREQ)) cmd_1_debouncer 
    (.clk(i_clk), .rst(1'b0), .btn_in(i_cmd_1), .btn_out(cmd_1_sync));
debouncer #(.CLK_FREQ(CLK_FREQ)) cmd_2_debouncer 
    (.clk(i_clk), .rst(1'b0), .btn_in(i_cmd_2), .btn_out(cmd_2_sync));

/* Oled Reset Timer */
localparam integer RESET_TIMER_MAX_COUNT = (0.01 * CLK_FREQ); // 10ms reset time
localparam integer RESET_TIMER_WIDTH     = $clog2(RESET_TIMER_MAX_COUNT + 1);
wire rst_cnt_reached;
reg rst_cnt_start = 1'b0;
flex_counter #(.MAX_COUNT(RESET_TIMER_MAX_COUNT), .WIDTH(RESET_TIMER_WIDTH)) reset_timer 
    (.clk(i_clk), .rst(i_rst), .cen(rst_cnt_start), .maxcnt(rst_cnt_reached), .count());

/* Instantiate the I2C Master */
wire [9:0] oled_addr;
assign oled_addr = OLED_DEV_ADDR;
reg [7:0] i2c_byte_cnt, i2c_tx_data;
reg [3:0] i2c_control_reg, i2c_mode_reg;
wire i2c_rx_data_valid, i2c_tx_data_needed;
wire [7:0] i2c_rx_data;
wire [4:0] i2c_status_reg;
i2c_master #(
    .SYS_CLOCK_FREQ_HZ(CLK_FREQ),
    .FAST_MODE_ENABLED(I2C_FAST_MODE_EN)
) i2c_master_core (
    .i_clk(i_clk),
    .i_rst(rst_sync),
    .i_slave_addr(oled_addr),
    .i_byte_cnt(i2c_byte_cnt),
    .i_control_reg(i2c_control_reg),
    .i_mode_reg(i2c_mode_reg),
    .i_tx_data(i2c_tx_data),
    .o_rx_data_valid(i2c_rx_data_valid),
    .o_tx_data_needed(i2c_tx_data_needed),
    .o_rx_data(i2c_rx_data),
    .o_status_reg(i2c_status_reg),
    .io_sda(i2c_sda),
    .io_scl(i2c_scl)
);













always @(posedge i_clk or posedge rst_sync) begin

    if(rst_sync) begin
        
        
    end else begin


    end


end




































endmodule