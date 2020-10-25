#include "grbl/nvs.h"

nvs_transfer_result_t memcpy_to_eeprom(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum);
nvs_transfer_result_t memcpy_from_eeprom(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum);
