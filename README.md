//////////////////////////////////////////////////////////////
// SDRAM Controller for DE0-NANO
//////////////////////////////////////////////////////////////

Version: 0.1.0  
Author: Jordan Downie <jpjdownie.biz@gmail.com>  
License: BSD 2-Clause  

This project provides an SDRAM controller wrapper around
nullobject/sdram-fpga with a built-in verification process.
It is designed and tested for the DE0-NANO FPGA development board.

//////////////////////////////////////////////////////////////
// 1. Features
//////////////////////////////////////////////////////////////

1) SDRAM controller instantiation using nullobject/sdram-fpga
2) Automatic read / write / verify process built into top level
3) LED status output showing progress and validation percentage
4) PLL instantiation for generating SDRAM and system clocks

//////////////////////////////////////////////////////////////
// 2. Top-Level Entity
//////////////////////////////////////////////////////////////

Entity: fpga_top

Inputs:
  - clk : main input clock

SDRAM interface:
  - dram_addr   (address bus)
  - dram_dq     (data bus)
  - dram_ba     (bank select)
  - dram_dqm    (data mask)
  - dram_ras_n, dram_cas_n, dram_we_n, dram_cs_n (control)
  - dram_clk, dram_cke (clock and enable)

Outputs:
  - LEDS_n (8-bit active-low LEDs for status display)

//////////////////////////////////////////////////////////////
// 3. Verification Method
//////////////////////////////////////////////////////////////

The design includes a self-test routine to check SDRAM operation.

Step 1: Write Phase
  - Each SDRAM address is written with its own address value.

Step 2: Read Phase
  - After writing, each address is read back sequentially.

Step 3: Validation
  - Read data is compared against the expected address.
  - A validation counter increments on success.
  - Mismatches do not increment the counter.

Step 4: LED Display
  - During writing: LEDs show the current write address (low bits).
  - During reading: LEDs show current read address and last data (low bits).
  - After completion: LEDs show the percentage of successful validations.

//////////////////////////////////////////////////////////////
// 4. Constants
//////////////////////////////////////////////////////////////

- c_max : maximum number of tested addresses = 8,388,607
- SDRAM timings (CAS latency, refresh, etc.) are configured in the SDRAM instantiation

//////////////////////////////////////////////////////////////
// 5. Dependencies
//////////////////////////////////////////////////////////////

- nullobject/sdram-fpga (SDRAM controller core)
- pll0 (Quartus-generated PLL, 50 MHz input clock)

//////////////////////////////////////////////////////////////
// 6. Usage
//////////////////////////////////////////////////////////////

1) Clone repository and add fpga_top.vhd plus SDRAM sources to Quartus project
2) Include pll0 module generated for DE0-NANO (50 MHz base clock)
3) Compile and program to DE0-NANO board
4) Observe LEDs for test progress and validation status

//////////////////////////////////////////////////////////////
// 7. License
//////////////////////////////////////////////////////////////

This project is licensed under the BSD 2-Clause License.
See source headers for details.
