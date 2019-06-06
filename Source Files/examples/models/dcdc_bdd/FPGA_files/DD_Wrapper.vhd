--////////////////////////////////////////////////////////////////////////////////
--
-- Create Date: 2019-06-05.14:52:31
-- Module Name: DD
-- Project Name: Symbolic Controller implementation
-- Target Devices: MyRIO FPGA
--
-- This file has been created based on the scripts from M. Khaled:
-- http://www.hcs.ei.tum.de and adapted to MyRIO FPGA
--
--////////////////////////////////////////////////////////////////////////////////

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;


entity DD_Wrapper is
port(
pi00 : in STD_LOGIC;
pi01 : in STD_LOGIC;
pi02 : in STD_LOGIC;
pi03 : in STD_LOGIC;
pi04 : in STD_LOGIC;
pi05 : in STD_LOGIC;
pi06 : in STD_LOGIC;
pi07 : in STD_LOGIC;
pi08 : in STD_LOGIC;
pi09 : in STD_LOGIC;
pi10 : in STD_LOGIC;
pi11 : in STD_LOGIC;
pi12 : in STD_LOGIC;
pi13 : in STD_LOGIC;
pi14 : in STD_LOGIC;
pi15 : in STD_LOGIC;

po0 : out STD_LOGIC;
po1 : out STD_LOGIC 
);
end DD_Wrapper;

architecture Behavioral of DD_Wrapper is

component DD
port (
pi00 : in STD_LOGIC;
pi01 : in STD_LOGIC;
pi02 : in STD_LOGIC;
pi03 : in STD_LOGIC;
pi04 : in STD_LOGIC;
pi05 : in STD_LOGIC;
pi06 : in STD_LOGIC;
pi07 : in STD_LOGIC;
pi08 : in STD_LOGIC;
pi09 : in STD_LOGIC;
pi10 : in STD_LOGIC;
pi11 : in STD_LOGIC;
pi12 : in STD_LOGIC;
pi13 : in STD_LOGIC;
pi14 : in STD_LOGIC;
pi15 : in STD_LOGIC;

po0 : out STD_LOGIC;
po1 : out STD_LOGIC 
);
end component;

begin

DD_x : DD
port map (
pi00 => pi00,
pi01 => pi01,
pi02 => pi02,
pi03 => pi03,
pi04 => pi04,
pi05 => pi05,
pi06 => pi06,
pi07 => pi07,
pi08 => pi08,
pi09 => pi09,
pi10 => pi10,
pi11 => pi11,
pi12 => pi12,
pi13 => pi13,
pi14 => pi14,
pi15 => pi15,

po0 => po0,
po1 => po1 
);

end Behavioral;
