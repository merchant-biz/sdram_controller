--------------------------------------------------------------------------------
-- SDRAM controller using nullobject/sdram-fpga with a simple verification 
-- process (designed for DE0-NANO)
--
-- @version 0.1.0
--
-- @author Jordan Downie <jpjdownie.biz@gmail.com>
-- @section LICENSE
-- 
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER ``AS IS'' AND ANY EXPRESS
-- OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
-- OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
-- NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
-- DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
-- ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
-- THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;

use work.pll0;
use work.sdram;

entity fpga_top is
    port (
		  -- CLK
        clk       : in  std_logic;
		  -- SDRAM
        dram_addr   : out unsigned(12 downto 0);
        dram_dq     : inout std_logic_vector(15 downto 0);
        dram_ba     : out unsigned(1 downto 0);
        dram_dqm    : out std_logic_vector(1 downto 0);
        dram_ras_n  : out std_logic;
        dram_cas_n  : out std_logic;
        dram_cke    : out std_logic;
        dram_clk    : out std_logic;
        dram_we_n   : out std_logic;
        dram_cs_n   : out std_logic;
		  -- LEDs
        LEDS_n      : out std_logic_vector(7 downto 0)
    );
end entity fpga_top;

architecture rtl of fpga_top is

    -- Constants
    constant c_max       : natural := 8_388_607;
    constant c_timer_max : unsigned(1 downto 0) := (others => '1');

    -- Clock & Reset Signals
    signal clkmain    : std_logic;
    signal pll_locked : std_logic;
    signal reset      : std_logic := '0';

    -- RAM Control Signals
    signal go_rd : std_logic := '0';
    signal go_wr : std_logic := '0';
	 
    signal ram_ctrl_addr  : unsigned(22 downto 0);
    signal ram_ctrl_data  : std_logic_vector(31 downto 0);
    signal ram_ctrl_we    : std_logic;
    signal ram_ctrl_req   : std_logic;
    signal ram_ctrl_ack   : std_logic;
    signal ram_ctrl_valid : std_logic;
    signal ram_ctrl_q     : std_logic_vector(31 downto 0);

    -- Read & Write Signals for RAM
    signal wr_ram_ctrl_addr : unsigned(22 downto 0);
    signal wr_ram_ctrl_data : std_logic_vector(31 downto 0);
    signal wr_ram_ctrl_we : std_logic;
    signal wr_ram_ctrl_req   : std_logic;

    signal rd_ram_ctrl_addr : unsigned(22 downto 0);
    signal rd_ram_ctrl_data : std_logic_vector(31 downto 0);
    signal rd_ram_ctrl_we   : std_logic;
    signal rd_ram_ctrl_req  : std_logic;

    -- State Machines
    type main_state_type is (IDLE, WRITING_CMD, WRITING_ACK, DONE);
    signal main_state : main_state_type := IDLE;

    type reading_state_type is (IDLE, READING_CMD, READING_ACK, DONE);
    signal reading_state : reading_state_type := IDLE;

    -- Counters
    signal vcnt  : natural range 0 to 8_388_607 := 0;
    signal wrcnt : natural range 0 to 8_388_607 := 0;
    signal rdcnt : natural range 0 to 8_388_607 := 0;
    signal timer : unsigned(1 downto 0);
    
    -- Data
    signal last_read_data : std_logic_vector(31 downto 0) := (others => '0');

begin

-- LEDs
-- Once writes and reads are complete,
-- print validation as a percentage onto LEDs
u_leds : process(clkmain, reset)
	variable tmp : integer := 0;
begin
	if rising_edge(clkmain) and pll_locked = '1' then
		if wrcnt < c_max then
			LEDS_n <= std_logic_vector(to_unsigned(wrcnt, 23))(7 downto 0);
		elsif rdcnt < c_max then
			if (reading_state = DONE) then
				LEDs_n <= std_logic_vector(to_unsigned(rdcnt, 23))(3 downto 0) & last_read_data(3 downto 0);
			end if;
		else
			if vcnt > 0 then
				tmp := 100 - ((c_max*100 - vcnt*100) / c_max);
			end if;
			LEDs_n <= std_logic_vector(to_unsigned(tmp, 8));
		end if;
	end if;
end process;

-- Main process
-- Write every address with its address
-- Read every address after writes and validate
u_main : process(clkmain)
begin 
	if rising_edge(clkmain) and pll_locked = '1' then
		if wrcnt < c_max then
			go_rd <= '0';
			go_wr <= '1';
			wr_ram_ctrl_addr <= to_unsigned(wrcnt, 23);
			wr_ram_ctrl_data <= std_logic_vector(to_unsigned(wrcnt,32))(31 downto 0);
			if (main_state = DONE) then
				wrcnt <= wrcnt + 1;
			end if;
		elsif rdcnt < c_max then
			go_rd <= '1';
			go_wr <= '0'; 
			rd_ram_ctrl_addr <= to_unsigned(rdcnt, 23);
			if (reading_state = DONE) then
				rdcnt <= rdcnt + 1;
				if (unsigned(last_read_data)(15 downto 0) = to_unsigned(rdcnt, 23)(15 downto 0)) then
					vcnt <= vcnt + 1;
				end if;
			end if;
		else
			go_wr <= '0';
			go_rd <= '0';
		end if;
	end if;
end process u_main;

-- PLL Instantiation
u_pll0 : entity pll0
port map(
	inclk0  => clk,
	c0      => clkmain,
	c1      => dram_clk,
	locked  => pll_locked
);

-- SDRAM
u_sdram_write : process(clkmain, reset)
begin
	if rising_edge(clkmain) and pll_locked = '1' then
		case main_state is
			when IDLE =>
				if go_wr = '1' then
					main_state <= WRITING_CMD;
				end if;
			when WRITING_CMD =>
				wr_ram_ctrl_we <= '1';
				wr_ram_ctrl_req <= '1';
				main_state <= WRITING_ACK;
			when WRITING_ACK =>
				if ram_ctrl_ack = '1' then
					wr_ram_ctrl_we <= '0';
					wr_ram_ctrl_req <= '0';
					main_state <= DONE;
				end if;
			when DONE =>
				main_state <= IDLE;
			when others =>
		end case;
	end if;
end process u_sdram_write;

u_sdram_read : process(clkmain)
begin
	if rising_edge(clkmain) and pll_locked = '1' and (main_state = IDLE) then
		case reading_state is
			when IDLE =>
				if go_rd = '1' then
					reading_state <= READING_CMD;
				end if;
			when READING_CMD =>
				if ram_ctrl_ack = '1' then
					rd_ram_ctrl_req <= '0';
					reading_state <= READING_ACK;
				else
					rd_ram_ctrl_we <= '0';
					rd_ram_ctrl_req <= '1';
				end if;
			when READING_ACK =>
				if ram_ctrl_valid = '1' then
					last_read_data <= ram_ctrl_q;
					reading_state <= DONE;
				end if;
			when DONE =>
				reading_state <= IDLE;
		end case;
	end if;
end process u_sdram_read;

-- Handles direction
u_sdram_bidir : process(main_state)
begin
	if main_state = WRITING_CMD or main_state = WRITING_ACK then
		ram_ctrl_addr <= wr_ram_ctrl_addr;
		ram_ctrl_data <= wr_ram_ctrl_data;
		ram_ctrl_we   <= wr_ram_ctrl_we  ;
		ram_ctrl_req  <= wr_ram_ctrl_req ;
	elsif main_state = IDLE then
		ram_ctrl_addr <= rd_ram_ctrl_addr;
		ram_ctrl_data <= (others => 'Z');
		ram_ctrl_we   <= rd_ram_ctrl_we  ;
		ram_ctrl_req  <= rd_ram_ctrl_req ;
	end if;
end process u_sdram_bidir;

-- Handles SDRAM state-machine
u_sdram : entity sdram
generic map(
    CAS_LATENCY =>        2,
    CLK_FREQ    =>     50.0,
    T_DESL      => 200000.0,
    T_MRD       =>     14.0,
    T_RC        =>     60.0,
    T_RCD       =>     15.0,
    T_RP        =>     15.0,
    T_WR        =>     15.0,
    T_REFI      =>  20000.0
)
port map(
    reset       =>  not pll_locked,
    clk         =>  clkmain,
    addr        =>  ram_ctrl_addr,
    data        =>  ram_ctrl_data,
    we          =>  ram_ctrl_we,
    req         =>  ram_ctrl_req,
    ack         =>  ram_ctrl_ack,
    valid       =>  ram_ctrl_valid,
    q           =>  ram_ctrl_q,

    sdram_a     =>  dram_addr,
    sdram_ba    =>  dram_ba,
    sdram_dq    =>  dram_dq,
    sdram_cke   =>  dram_cke,
    sdram_cs_n  =>  dram_cs_n,
    sdram_ras_n =>  dram_ras_n,
    sdram_cas_n =>  dram_cas_n,
    sdram_we_n  =>  dram_we_n,
    sdram_dqml  =>  dram_dqm(0),
    sdram_dqmh  =>  dram_dqm(1)
);

end architecture rtl;