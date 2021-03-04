/*******************************************************************************
    Verilog netlist generated by IPGEN Lattice Radiant Software (64-bit)
    2.2.0.97.3
    Soft IP Version: 1.1.1
    2021 03 04 00:32:38
*******************************************************************************/
/*******************************************************************************
    Wrapper Module generated per user settings.
*******************************************************************************/
module dpram512x8 (wr_clk_i, rd_clk_i, rst_i, wr_clk_en_i, rd_en_i,
    rd_clk_en_i, wr_en_i, wr_data_i, wr_addr_i, rd_addr_i, rd_data_o)/* synthesis syn_black_box syn_declare_black_box=1 */;
    input  wr_clk_i;
    input  rd_clk_i;
    input  rst_i;
    input  wr_clk_en_i;
    input  rd_en_i;
    input  rd_clk_en_i;
    input  wr_en_i;
    input  [35:0]  wr_data_i;
    input  [8:0]  wr_addr_i;
    input  [8:0]  rd_addr_i;
    output  [35:0]  rd_data_o;
endmodule