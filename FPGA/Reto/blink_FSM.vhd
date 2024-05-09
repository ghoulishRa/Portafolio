library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;


Entity blink_FSM is

	Port(
		processing : in std_logic_vector(7 downto 0);
		clk_in: in std_logic;
		HS, VS : out std_logic;
		Red,Green, Blue: out std_logic_vector (3 downto 0)
		);
		
End blink_FSM;


Architecture behavior of blink_FSM is

	signal clk_25, HS_temp, VS_temp : std_logic; -- señales para pintar en X y en Y
	signal display_HS, display_VS : std_logic; -- 
	signal signal_posX : std_logic_vector(9 downto 0);
	signal signal_posY : std_logic_vector (9 downto 0);
	
	
	type state_type is (s0, s1, s2, s3);
	signal estados : state_type;
	
	
Begin

	--maquina de estados

	process (processing, clk_in)
	begin
	
		if (rising_edge(clk_in)) then
			case estados is
				when s0 =>  -- apagado / s0
					if (processing = ("00110011"))then
						estados <= s3;
					else
						estados <= estados;
					end if;
				when s1 => -- una vida / s1  
					if (processing = ("00110000")) then
						estados <= s0;
					else
						estados <= estados;
					end if;
				when s2 => -- dos vida / s2
					if (processing = ("00110001")) then
						estados <= s1;
					else
						estados <= estados;
					end if;
				when s3 => --tres vidas
					if (processing = "00110010") then
						estados <= s2;
					else
						estados <= estados;
					end if;
			end case;
		end if;
 
	end process;


	process(clk_in)
	Begin
	
		if(rising_edge(clk_in)) then
			clk_25 <= not (clk_25);
		else
			clk_25 <= clk_25;
		End if;
	End process;
	
	--96, 144, 784, 800
	
	process (clk_25) -- Señal HS
		variable counter : integer := 0;
		variable posx : integer := 0;
	begin
		if (rising_edge(clk_25)) then
			counter := counter + 1;
			if (counter < 96) then -- pulse width
				HS_temp <= '0';
				display_HS <= '0';
			elsif (counter < 144) then -- back porch
				HS_temp <= '1';
				display_HS <= '0';
			elsif (counter < 784) then -- display time
				HS_temp <= '1';
				display_HS <= '1';
				posx := posx + 1;
			elsif (counter < 800) then -- front porch
				HS_temp <= '1';
				display_HS <= '0';
			else
				HS_temp <= '0';
				counter := 0;
				posx := 0;
				display_HS <= '0'; -- sección de reinicio de variables
			end if;
		else
			HS_temp <= HS_temp;
		end if;
		
		signal_posX <= std_logic_vector(to_unsigned(posx,10));
		
	end process;
	
	-- 2, 31, 511, 521
	
	process (HS_temp)	-- Señal VS
	 variable lineas : integer := 0;
	 variable posy : integer := 0;
	begin
		if (falling_edge(HS_temp)) then
			lineas := lineas + 1;
			if (lineas < 2) then
				VS_temp <= '0';
				display_VS <= '0';
			elsif (lineas < 31) then
				VS_temp <= '1';
				display_VS <= '0';
			elsif (lineas < 511) then --display time
				VS_temp <= '1';
				display_VS <= '1';
				posy := posy + 1;
			elsif (lineas < 521) then
				VS_temp <= '1';
				display_VS <= '0';
			else
				VS_temp <= '0'; --sección de reinicio de variables
				lineas := 0;
				posy := 0;
				display_VS <= '0';
			end if;
		else
			VS_temp <= VS_temp;
		end if;
		
		signal_posY <= std_logic_vector(to_unsigned(posy,10));
		
	end process;
	
	--controlador RGB
	
	process(signal_posX,signal_posY,clk_25)
	
	begin
	
	
	--fila 1
	
		if((signal_posX > 40 and signal_posX < 80) and (signal_posY > 120 and signal_posY < 140) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 140   and signal_posX < 180) and (signal_posY > 120 and signal_posY < 140) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 240   and signal_posX < 280) and (signal_posY > 120 and signal_posY < 140) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 340   and signal_posX < 380) and (signal_posY > 120 and signal_posY < 140) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif((signal_posX > 440   and signal_posX < 480) and (signal_posY > 120 and signal_posY < 140) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif((signal_posX > 540   and signal_posX < 580) and (signal_posY > 120 and signal_posY < 140) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
			
		--fila  2
		
		elsif( (signal_posX > 20   and signal_posX < 100) and (signal_posY > 140 and signal_posY < 160) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 120   and signal_posX < 200) and (signal_posY > 140 and signal_posY < 160) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 220   and signal_posX < 300) and (signal_posY > 140 and signal_posY < 160) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 320   and signal_posX < 400) and (signal_posY > 140 and signal_posY < 160) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 420   and signal_posX < 500) and (signal_posY > 140 and signal_posY < 160) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 520   and signal_posX < 600) and (signal_posY > 140 and signal_posY < 160) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		
		--fila 3
		
		elsif( (signal_posX > 20   and signal_posX < 200) and (signal_posY > 160 and signal_posY < 180) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 220   and signal_posX < 400) and (signal_posY > 160 and signal_posY < 180) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 420   and signal_posX < 600) and (signal_posY > 160 and signal_posY < 180) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
			
			
		--fila 4
		
		
		elsif( (signal_posX > 40  and signal_posX < 180) and (signal_posY > 180 and signal_posY < 200) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 240  and signal_posX < 380) and (signal_posY > 180 and signal_posY < 200) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif( (signal_posX > 440  and signal_posX < 580) and (signal_posY > 180 and signal_posY < 200) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
			
		--fila 5
		
		elsif ((signal_posX > 60 and signal_posX < 160) and (signal_posY > 200 and signal_posY < 220) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif ((signal_posX > 260  and signal_posX < 360) and (signal_posY > 200 and signal_posY < 220) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif ((signal_posX > 460 and signal_posX < 560 )and (signal_posY > 200 and signal_posY < 220) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
			
		-- fila 6 
		
		elsif((signal_posX > 80 and signal_posX < 140) and (signal_posY > 220 and signal_posY < 240) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif((signal_posX > 280  and signal_posX < 340) and (signal_posY > 220 and signal_posY < 240) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif ((signal_posX  > 480 and signal_posX < 540)  and (signal_posY > 220 and signal_posY < 240) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
			
			
			
		-- fila 7
		
		elsif((signal_posX > 100 and signal_posX < 120) and (signal_posY > 240 and signal_posY < 260) and (estados = s3)) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif((signal_posX > 300  and signal_posX < 320) and (signal_posY > 240 and signal_posY < 260) and ((estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";
		elsif ((signal_posX > 500 and signal_posX < 520) and (signal_posY > 240 and signal_posY < 260) and ((estados = s1) or (estados = s2) or (estados = s3))) then
			Red <= "1111";
			Green <= "0000";
			Blue <= "0000";																																																																																																															
		else
			Red <= "0000";
			Green <= "0000";
			Blue <= "0000";
			
		end if;
		
	end process;
	
	--Asignas salidas temporales
	
	HS <= HS_temp; 
	VS <= VS_temp;
	
end behavior;

