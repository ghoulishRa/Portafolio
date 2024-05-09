--Equipo 3.
--Código integrador del reto.

LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;


Entity pJoy is
	Port(
		--Clock
		clk : IN STD_LOGIC; --igual sirve para txrx
		rst : IN STD_LOGIC; --reset de datos
		
	
		--Entradas y salidas para Tx (12 bits) y para Rx (8 bits)
		tEnable : in std_logic;
		rX : in std_logic;
		rstTxRx : in std_logic;
		tX : out std_logic;
		
		--VGA
		VerS, HorS : out std_logic;
		Red, Green, Blue : out std_logic_vector(3 downto 0)
		
		
	);
END pJoy;

Architecture Behavioral of pJoy is 
	--ADC
	Signal s_ch0	: std_logic_vector (11 downto 0);
	Signal s_ch1	: std_logic_vector (11 downto 0);
	Signal s_ch2	: std_logic_vector (11 downto 0);
	Signal s_ch3	: std_logic_vector (11 downto 0);
	Signal s_ch4	: std_logic_vector (11 downto 0);
	Signal s_ch5	: std_logic_vector (11 downto 0);
	Signal s_ch6	: std_logic_vector (11 downto 0);
	Signal s_ch7	: std_logic_vector (11 downto 0);
	
	
	--Reloj de 9600 Baudios
	signal clk_div : std_logic;
	
	--Señal rx 11 bits temporal
	signal recX: std_logic;
	
	--Señal 7 segmentos temporal
	signal segTemp : std_logic_vector(6 downto 0);
	

	component ADC_2 is
		port (
			CLOCK : in  std_logic                     := '0'; --      clk.clk
			CH0   : out std_logic_vector(11 downto 0);        -- readings.CH0
			CH1   : out std_logic_vector(11 downto 0);        --         .CH1
			CH2   : out std_logic_vector(11 downto 0);        --         .CH2
			CH3   : out std_logic_vector(11 downto 0);        --         .CH3
			CH4   : out std_logic_vector(11 downto 0);        --         .CH4
			CH5   : out std_logic_vector(11 downto 0);        --         .CH5
			CH6   : out std_logic_vector(11 downto 0);        --         .CH6
			CH7   : out std_logic_vector(11 downto 0);        --         .CH7
			RESET : in  std_logic                     := '0'  --    reset.reset
		);
	end component ADC_2; 
	
	component Full_serial is
		port (clk_fpga      	: in std_logic;
				enable_fpga    : in std_logic;
				Rx_fpga      	: in std_logic;
				reset       	: in std_logic;
				databits_fpga	: in std_logic_vector (11 downto 0);
				Tx_fpga	      : out std_logic
		);
		
	end component;
	
	component vidasFSM is
	port(
		clk : in std_logic;
		HS, VS : out std_logic;
		R,G,B: out std_logic_vector (3 downto 0);
		dis: out std_logic_vector(6 downto 0);
		rX : in std_logic
		);
	end component;
	
	Begin
	
	process(clk)
		variable count_clk : integer := 0;
	
	begin
		if rising_edge(clk) then
			count_clk := count_clk + 1;
			if count_clk = (50000000/(1)) then
				clk_div <= not clk_div;
				count_clk := 0;
			end if;
		end if;
	end process;

		adc2 : ADC_2 port map(
				clk,
				s_ch0,
				s_ch1,
				s_ch2,
				s_ch3,
				s_ch4,
				s_ch5,
				s_ch6,
				s_ch7,
				not rst
			);
		

		transmision: Full_serial port map (clk, tEnable, recX, rstTxRx, s_ch0, tX);
		recepcionVidas: vidasFSM port map(clk, HorS, VerS, Red, Green, Blue, segTemp, rX);

End Behavioral;


