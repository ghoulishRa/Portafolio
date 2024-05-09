library ieee;
use ieee.std_logic_1164.all;

entity Tx_RX is
	port (clk		: in std_logic;
			enable   : in std_logic;
			Rx       : in std_logic;
			reset    : in std_logic;
			dataBit  : in std_logic_vector (11 downto 0);
			data_out	: out std_logic_vector(11 downto 0);
			Tx       : out std_logic
	);
end Tx_RX;

Architecture behavior of Tx_RX is

	signal clk_div : std_logic;
	signal data : std_logic_vector(11 downto 0);
	
	type states_Tx is (T0,T1,T2);
   signal stateTx : states_Tx;
	 
	type states_Rx is (R0,R1,R2);
   signal stateRx : states_Rx;

   
begin
	process(clk)
		variable count_clk : integer := 0;
	
	begin
		if rising_edge(clk) then
			count_clk := count_clk + 1;
			if count_clk = (50000000/(2*9600)) then
				clk_div <= not clk_div;
				count_clk := 0;
			end if;
		end if;
	end process;
	 

	 --Tx
	process(clk_div,enable)
		variable count : integer := 0;
		variable data_copy : std_logic_vector(11 downto 0);

   begin
		if rising_edge(clk_div) then
			case stateTx is
				when T0 =>
					
					data_copy := dataBit;
					Tx <= '0';
					stateTx <= T1;
			  
				when T1 =>
					if (count = 12) then
						count := 0;
						stateTx <= T2;
					else
						Tx <= data_copy(count);
						count := count + 1;
						stateTx <= stateTx;
					end if;

				when T2 =>
					
						Tx <= '1';
						stateTx <= T0;
					
			end case;
		end if;
	end process;

	 
	--Rx
	process(clk_div, Rx)
		variable count : integer := 0;
		variable data_copy : std_logic_vector(11 downto 0);

	begin
		if rising_edge(clk_div) then
			case stateRx is
				when R0 =>
					if Rx = '0' then
						stateRx <= R1;
					else
						stateRx <= stateRx;						  
					end if;					 
					
				when R1 =>
					if (count = 12) then
						count := 0;
						data <= data_copy;
						stateRx <= R2;
					else
						data_copy(count) := Rx;
						count := count + 1;
						stateRx <= stateRx;
					end if;
					
				when R2 =>
					if Rx = '1' then
						stateRx <= R0;
					else
						stateRx <= R0;
					end if;
					
			end case;
		end if;		
	end process;
	
	data_out <= data;
	
end behavior;