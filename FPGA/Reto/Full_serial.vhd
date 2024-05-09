library ieee;
use ieee.std_logic_1164.all;

entity Full_serial is
	port (clk_fpga      	: in std_logic;
			enable_fpga    : in std_logic;
			Rx_fpga      	: in std_logic;
			reset       	: in std_logic;
			databits_fpga	: in std_logic_vector (11 downto 0);
			Tx_fpga	      : out std_logic
	);
	
end Full_serial;

Architecture behavior of Full_serial is

	component Tx_Rx is
		port (clk		: in std_logic;
				enable   : in std_logic;
				Rx       : in std_logic;
				reset    : in std_logic;
				dataBit 	: in std_logic_vector (11 downto 0);
				data_out : out std_logic_vector(11 downto 0);
				Tx       : out std_logic
	);
	end component;
	
	
	signal data_temp : std_logic_vector(11 downto 0); 
	
begin
	
	U0: Tx_Rx port map(clk_fpga, enable_fpga, Rx_fpga, reset, databits_fpga, data_temp, Tx_fpga);

end behavior;