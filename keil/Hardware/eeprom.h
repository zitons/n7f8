#ifndef __EEPROM_H
#define __EEPROM_H

EEPROM_ByteWrite(uint8_t addr, uint8_t data);
EEPROM_RandomRead(uint8_t addr, uint8_t* data);


#endif