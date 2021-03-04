`include "svut_h.sv"
`timescale 1 ns / 1 ps

module async_fifo_unit_test;

    `SVUT_SETUP

    integer i;

    parameter DSIZE = 32;
    parameter ASIZE = 4;

    reg              wclk;
    reg              wrst_n;
    reg              winc;
    reg  [DSIZE-1:0] wdata;
    wire             wfull;
    wire             awfull;
    reg              rclk;
    reg              rrst_n;
    reg              rinc;
    wire [DSIZE-1:0] rdata;
    wire             rempty;
    wire             arempty;

    async_fifo 
    #(
    DSIZE,
    ASIZE
    )
    dut 
    (
    wclk,
    wrst_n,
    winc,
    wdata,
    wfull,
    awfull,
    rclk,
    rrst_n,
    rinc,
    rdata,
    rempty,
    arempty
    );

    // An example to create a clock
    initial wclk = 1'b0;
    always #2 wclk <= ~wclk;
    initial rclk = 1'b0;
    always #3 rclk <= ~rclk;

    // An example to dump data for visualization
    initial begin
        $dumpvars(0, async_fifo_unit_test);
    end

    task setup();
    begin

        wrst_n = 1'b0;
        winc = 1'b0;
        wdata = 0;
        rrst_n = 1'b0;
        rinc = 1'b0;
        #100;
        wrst_n = 1;
        rrst_n = 1;
        #200;
        @(posedge wclk);

    end
    endtask

    task teardown();
    begin

        #200;

    end
    endtask

    `UNIT_TESTS

    `UNIT_TEST(IDLE)
        
        `INFO("Test: IDLE");
        `FAIL_IF(wfull);
        `FAIL_IF(!rempty);
    
    `UNIT_TEST_END

    `UNIT_TEST(SIMPLE_WRITE_AND_READ)

        `INFO("Test: Simple write then read");
        
        @(posedge wclk)

        winc = 1;
        wdata = 32'hA;

        @(posedge wclk)

        winc = 0;

        @(posedge rclk)

        wait (rempty == 1'b0);

        `FAIL_IF_NOT_EQUAL(rdata, 32'hA);

    `UNIT_TEST_END

    `UNIT_TEST(MULTIPLE_WRITE_AND_READ)

        `INFO("Test: Multiple write then read");
        
        for (i=0; i<10; i=i+1) begin
            @(negedge wclk);
            winc = 1;
            wdata = i;
            // $display("DEBUG:   [%g]: %x", $time, i);
        end
        @(negedge wclk);
        winc = 0;
        
        #100;

        @(posedge rclk);

        rinc = 1;
        for (i=0; i<10; i=i+1) begin
            @(posedge rclk);
            `FAIL_IF_NOT_EQUAL(rdata, i);
            // $display("DEBUG:   [%g]: %x", $time, rdata);
        end

    `UNIT_TEST_END

    `UNIT_TEST(TEST_FULL_FLAG)

        `INFO("Test: full flag test");
        
        winc = 1;

        for (i=0; i<2**ASIZE; i=i+1) begin
            @(negedge wclk)
            wdata = i;
        end

        @(negedge wclk);
        winc = 0;

        @(posedge wclk)
        `FAIL_IF_NOT_EQUAL(wfull, 1);

    `UNIT_TEST_END

    `UNIT_TEST(TEST_EMPTY_FLAG)
        
        `INFO("Test: empty flag test");

        `FAIL_IF_NOT_EQUAL(rempty, 1);
        
        for (i=0; i<2**ASIZE; i=i+1) begin
            @(posedge wclk)
            winc = 1;
            wdata = i;
        end

        `FAIL_IF_NOT_EQUAL(rempty, 0);

    `UNIT_TEST_END

    `UNIT_TEST(TEST_SIMPLE_ALMOST_EMPTY_FLAG)
        
        `INFO("Test: almost empty flag simple test");
        
        `FAIL_IF_NOT_EQUAL(arempty, 0);

        @(posedge wclk)
        winc = 1;
        wdata = i;
        @(posedge wclk);
        winc = 0;

        #100;
        `FAIL_IF_NOT_EQUAL(arempty, 1);

    `UNIT_TEST_END

    `UNIT_TEST(TEST_SIMPLE_ALMOST_FULL_FLAG)

        `INFO("Test: Almost full flag simple test");
        
        winc = 1;
        for (i=0; i<2**ASIZE; i=i+1) begin

            @(negedge wclk)
            wdata = i;

        end

        @(negedge wclk);
        winc = 0;

        @(posedge wclk)
        `FAIL_IF_NOT_EQUAL(wfull, 1);
    
    `UNIT_TEST_END

    `UNIT_TEST(TEST_CONSECUTIVE_ALMOST_EMPTY_FULL)

        `INFO("Test: Consecutive empty/full flags test");
        
        winc = 1;
        for (i=0; i<2**ASIZE; i=i+1) begin

            @(negedge wclk)
            wdata = i;

        end

        @(negedge wclk);
        winc = 0;

        @(posedge wclk)
        `FAIL_IF_NOT_EQUAL(wfull, 1);
    
    `UNIT_TEST_END
    `UNIT_TESTS_END

endmodule

