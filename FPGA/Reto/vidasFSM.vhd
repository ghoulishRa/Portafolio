library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

entity vidasFSM is
port(
	clk : in std_logic;
	HS, VS : out std_logic;
	R,G,B: out std_logic_vector (3 downto 0);
	dis: out std_logic_vector(6 downto 0);
	rX : in std_logic
	
);
end vidasFSM;

architecture behav of vidasFSM is

component blink_FSM
	Port(
		processing : in std_logic_vector(7 downto 0);
		clk_in: in std_logic;
		HS, VS : out std_logic;
		Red,Green, Blue: out std_logic_vector (3 downto 0)
		);
end component;

component Tx_RX8 is
	port (clk		: in std_logic;
			enable   : in std_logic;
			Rx       : in std_logic;
			reset    : in std_logic;
			dataBit  : in std_logic_vector (7 downto 0);
			data_out	: out std_logic_vector(7 downto 0);
			Tx       : out std_logic
	);
end component;

signal en,res,tx : std_logic;
signal dain, dataProcessing : std_logic_vector(7 downto 0);

begin

	u0 : Tx_RX8 port map (clk, en, rX, res, dain, dataProcessing, tx);
	u1 : blink_FSM port map (dataProcessing, clk, HS, VS, R, G , B);
	
end behav;