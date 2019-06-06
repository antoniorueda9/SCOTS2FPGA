--////////////////////////////////////////////////////////////////////////////////
--
-- Create Date: #$DATES$#
-- Module Name: #$ENTITY_MODEL_NAME$#
-- Project Name: Symbolic Controller implementation
-- Target Devices: MyRIO FPGA
--
-- This file has been created based on the scripts from M. Khaled:
-- http://www.hcs.ei.tum.de and adapted to MyRIO FPGA
--
--////////////////////////////////////////////////////////////////////////////////

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;


entity #$ENTITY_MODEL_NAME$#_Wrapper is
port(
#$ENTITY_INPUT_PORTS$#
#$ENTITY_OUTPUT_PORTS$#);
end #$ENTITY_MODEL_NAME$#_Wrapper;

architecture Behavioral of #$ENTITY_MODEL_NAME$#_Wrapper is

component #$ENTITY_MODEL_NAME$#
port (
#$ENTITY_INPUT_PORTS$#
#$ENTITY_OUTPUT_PORTS$#);
end component;

begin

#$ENTITY_MODEL_NAME$#_x : #$ENTITY_MODEL_NAME$#
port map (
#$ENTITY_INPUT_PORTS_MAP$#
#$ENTITY_OUTPUT_PORTS_MAP$#);

end Behavioral;
