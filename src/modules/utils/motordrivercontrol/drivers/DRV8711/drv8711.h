#pragma once

#include <functional>
#include <bitset>

class StreamOutput;

class DRV8711DRV
{
public:
  DRV8711DRV(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi);

  void init(uint16_t gain = 20) ;

  void set_enable(bool enable) ;
  int set_microsteps(int number_of_steps);
  void set_current(uint32_t currentma);

  void dump_status(StreamOutput *stream) ;
  bool set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val);
  bool check_alarm();

private:

  uint16_t ReadWriteRegister(uint8_t dataHi, uint8_t dataLo);
  uint16_t ReadRegister(uint8_t addr);
  void ReadAllRegisters () ;
  void WriteAllRegisters () ;

// WARNING this may not be portable, and endianess affects the order, but it works for LPC1769
// CTRL Register
  typedef union {
    struct {
      uint8_t ENBL: 1;   // bit 0
      uint8_t RDIR: 1;   // bit 1
      uint8_t RSTEP: 1;  // bit 2
      uint8_t MODE: 4;   // bits 6-3
      uint8_t EXSTALL: 1; // bit 7
      uint8_t ISGAIN: 2; // bits 9-8
      uint8_t DTIME: 2;  // bits 11-10
      uint8_t Address: 3; // bits 14-12
    };
    uint16_t raw;
  } CTRL_Register_t;

// TORQUE Register
  typedef union {
    struct {
      uint8_t TORQUE: 8;    // bits 7-0
      uint8_t SIMPLTH: 3;   // bits 10-8
      uint8_t Reserved: 1;  // bit 11
      uint8_t Address: 3;   // bits 14-12
    };
    uint16_t raw;
  } TORQUE_Register_t;

// OFF Register
  typedef union {
    struct {
      uint8_t TOFF: 8;    // bits 7-0
      uint8_t PWMMODE: 1; // bit 8
      uint8_t Reserved: 3; // bits 11-9
      uint8_t Address: 3; // bits 14-12
    };
    uint16_t raw;
  } OFF_Register_t;

// BLANK Register
  typedef union {
    struct  {
      uint8_t TBLANK:8;  // bits 7-0
      uint8_t ABT:1;     // bit 8
      uint8_t Reserved:3; // bits 11-9
      uint8_t Address:3; // bits 14-12
    };
    uint16_t raw;
  } BLANK_Register_t;

// DECAY Register
  typedef union {
    struct  {
      uint8_t TDECAY:8;  // bits 7-0
      uint8_t DECMOD:3;    // bits 10-8
      uint8_t Reserved:1; // bit 11
      uint8_t Address:3; // bits 14-12
    };
    uint16_t raw;
  } DECAY_Register_t;

// STALL Register
  typedef union {
    struct  {
      uint8_t SDTHR:8;   // bits 7-0
      uint8_t SDCNT:2;   // bits 9-8
      uint8_t VDIV:2;    // bits 11-10
      uint8_t Address:3; // bits 14-12
    };
    uint16_t raw;
  } STALL_Register_t;

// DRIVE Register
  typedef union {
    struct  {
      uint8_t OCPTH:2;   // bits 1-0
      uint8_t OCPDEG:2;  // bits 3-2
      uint8_t TDRIVEN:2; // bits 5-4
      uint8_t TDRIVEP:2; // bits 7-6
      uint8_t IDRIVEN:2; // bits 9-8
      uint8_t IDRIVEP:2; // bits 11-10
      uint8_t Address:3; // bits 14-12
    };
    uint16_t raw;
  } DRIVE_Register_t;

// STATUS Register
  typedef union {
    struct  {
      uint8_t OTS:1;   // bit 0
      uint8_t AOCP:1;    // bit 1
      uint8_t BOCP:1;    // bit 2
      uint8_t APDF:1;    // bit 3
      uint8_t BPDF:1;    // bit 4
      uint8_t UVLO:1;    // bit 5
      uint8_t STD:1;   // bit 6
      uint8_t STDLAT:1;    // bit 7
      uint8_t Reserved:4; // bits 11-8
      uint8_t Address:3; // bits 14-12
    };
    uint16_t raw;
  } STATUS_Register_t;

  CTRL_Register_t    G_CTRL_REG;
  TORQUE_Register_t  G_TORQUE_REG;
  OFF_Register_t     G_OFF_REG;
  BLANK_Register_t   G_BLANK_REG;
  DECAY_Register_t   G_DECAY_REG;
  STALL_Register_t   G_STALL_REG;
  DRIVE_Register_t   G_DRIVE_REG;
  STATUS_Register_t  G_STATUS_REG;

  std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi;
  float resistor{0.05};
  std::bitset<8> error_reported;
  uint8_t gain{20};


  // float _amps;
  // uint8_t _microstepreg;
};
