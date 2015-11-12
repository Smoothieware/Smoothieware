#pragma once

#include <functional>

#include "drv8711-registers.h"

class StreamOutput;

class DRV8711DRV
{
public:
  DRV8711DRV(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi);

  void init(unsigned int current, unsigned int microsteps, unsigned int gain=20) ;

  void set_enable(bool enable) ;
  void dump_status(StreamOutput *stream) ;

private:

  unsigned int SPI_DRV8711_ReadWrite(unsigned char dataHi, unsigned char dataLo);
  void ReadAllRegisters () ;
  void WriteAllRegisters () ;

  struct CTRL_Register    G_CTRL_REG;
  struct TORQUE_Register  G_TORQUE_REG;
  struct OFF_Register     G_OFF_REG;
  struct BLANK_Register   G_BLANK_REG;
  struct DECAY_Register   G_DECAY_REG;
  struct STALL_Register   G_STALL_REG;
  struct DRIVE_Register   G_DRIVE_REG;
  struct STATUS_Register  G_STATUS_REG;

  std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi;
  uint8_t _amps ;
  uint8_t _torque ;
  uint8_t _gain ;
  uint8_t _microstepreg ;
};
