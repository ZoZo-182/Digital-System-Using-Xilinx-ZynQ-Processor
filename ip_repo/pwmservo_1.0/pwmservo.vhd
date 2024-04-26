----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 04/08/2024 08:14:45 PM
-- Design Name: 
-- Module Name: pwmservo - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity pwmservo is
port(
clk         : in std_logic;
datain : in std_logic_vector(15 downto 0);
PWM0                     : out std_logic
);
end pwmservo;

architecture Behavioral of pwmservo is
signal counter : integer:=0;  

begin

 process(clk) 

    begin 

    if rising_edge(clk) then 
        if counter < 999999 - 1 then 
       counter <= counter + 1; 
    else 
       counter <= 0; 
   end if; 
end if;            
    end process; 
    
    -- comparator statements that drive the PWM signal
    PWM0 <= '0' when datain < std_logic_vector(to_unsigned(counter, datain'length)) else '1'; 

end Behavioral;
