#define GCONF        0x00    //Global config flags.  
  #define GSTAT        0x01    //Global status flags.
  #define IFCNT        0x02    //Interface transmission counter. This register becomes incremented with each successful UART interface write access.
  #define SLAVECONF    0x03    //SLAVEADDR = bits 0 to 7. SENDDELAY = bits 8 to 11.
  #define INP_OUT      0x04    //INPUT: Reads the state of all input pins available. OUTPUT: Sets the IO output pin polarity in UART mode.
  #define XCOMPARE     0x05    //Position  comparison  register.
  #define IHOLD_IRUN   0x10    //Driver current control.
  #define TPOWERDOWN   0x11    //Sets the delay timw after standstill of the motor to motor current power down. Time range is about 0-4 sec.
  #define TSTEP        0x12    //Actual measured time between two 1/256 microsteps derived from the step input frequency in units of 1/fCLK.
  #define TPWMTHRS     0x13    //Upper velocity for StealthChop voltage PWM mode.
  #define TCOOLTHRS    0x14    //This is the lower threshold velocity for switching on smart energy coolStep and stallGuard feature.
  #define THIGH        0x15    //Velocity setting allows velocity dependent swithing into a different chopper mode and fullstepping to maximize torque. Stall detection becomes switched off for 2-3 electrical periods whenever passing THIGH threshold to compensate for the effect of switching modes. 

  #define RAMPMODE     0x20    //ramp mode.
  #define XACTUAL      0x21    //Actual motor position.  
  #define VACTUAL      0x22    //Actual motor velocity from ramp generator.
  #define VSTART       0x23    //Motor start velocity.
  #define A1           0x24    //First acceleration between VSTART and V1.
  #define V1           0x25    //First acceleration / deceleration phase threshold velocity.
  #define AMAX         0x26    //Second acceleration between V1 and VMAX.
  #define VMAX         0x27    //Motion ramp target velocity. It can be changed any time during a motion.
  #define DMAX         0x28    //Deceleration between VMAX and V1.
  #define D1           0x2A    //Deceleration between V1 and VSTOP. Do not set 0 in positioning mode, even if V1=0!
  #define VSTOP        0x2B    //Motor stop velocity. Do not set 0 in positioning mode, minimum 10 recommended!
  #define TZEROCROSS   0x2C    //Defines the waiting time after ramping down to zero velocity before next movement or direction inversion can start. Time range is about 0 to 2 seconds.
  #define XTARGET      0x2D    //Target position for ramp mode. Write a new target position to this register in order to activate the ramp generator positioning in RAMPMODE=0. Initialize all velocity, acceleration and deceleration parameters before.


  #define VDCMIN       0x33    //
  #define SWMODE       0x34    //Switch mode configuration.
  #define RAMPSTAT     0x35    //Ramp status and switch event status.  
  #define XLATCH       0x36    //Ramp generator latch position, latches XACTUAL upon a programmable switch event.
  #define ENCMODE      0x38    //Encoder configuration and use of N channel.
  #define XENC         0x39    //Actual encoder position
  #define ENC_CONST    0x3A    //
  #define ENC_STATUS   0x3B    //
  #define ENC_LATCH    0x3C    //Encoder position X_ENC latched on N event.
    
  #define MSLUT0       0x60    //
  #define MSLUT1       0x61    //  
  #define MSLUT2       0x62    //
  #define MSLUT3       0x63    //
  #define MSLUT4       0x64    //
  #define MSLUT5       0x65    //
  #define MSLUT6       0x66    //
  #define MSLUT7       0x67    //
  #define MSLUTSEL     0x68    //This register defines four segments within each quarter MSLUT wave.
  #define MSLUTSTART   0x69    //
  #define MSCNT        0x6A    //Microstep counter. Indicates actual position in the microstep table for CUR_A. CUR_B uses an offset of 256 (2 phase motor).
  #define MSCURACT     0x6B    //
  #define CHOPCONF     0x6C    //chopper and driver configuration.
  #define COOLCONF     0x6D    //CoolStep smart current control register.
  #define DCCTRL       0x6E    //
  #define DRVSTATUS    0x6F    //StallGuard2 value and driver error flags
  #define PWMCONF      0x70    //Voltage PWM mode chopper configuration
  #define PWMSTATUS    0x71    //Actual PWM amplitude scaler (255=max. Voltage) In voltage mode PWM, this value allows to detect a motor stall.
  #define EN_CTRL      0x72    //
  #define LOST_STEPS   0x73    //
    
  //Rampenmodi (Register TMCRhino_RAMPMODE)
  #define MODE_POSITION   0
  #define MODE_VELPOS     1
  #define MODE_VELNEG     2
  #define MODE_HOLD       3

  //Endschaltermodusbits (Register TMCRhino_SWMODE)
  #define SW_STOPL_ENABLE   0x0001
  #define SW_STOPR_ENABLE   0x0002
  #define SW STOPL_POLARITY 0x0004
  #define SW_STOPR_POLARITY 0x0008
  #define SW_SWAP_LR        0x0010
  #define SW_LATCH_L_ACT    0x0020
  #define SW_LATCH_L_INACT  0x0040
  #define SW_LATCH_R_ACT    0x0080
  #define SW_LATCH_R_INACT  0x0100
  #define SW_LATCH_ENC      0x0200
  #define SW_SG_STOP        0x0400
  #define SW_SOFTSTOP       0x0800


  //Statusbitss (Register TMCRhino_RAMPSTAT)
  #define RS_STOPL          0x0001
  #define RS_STOPR          0x0002
  #define RS_LATCHL         0x0004
  #define RS_LATCHR         0x0008
  #define RS_EV_STOPL       0x0010
  #define RS_EV_STOPR       0x0020
  #define RS_EV_STOP_SG     0x0040
  #define RS_EV_POSREACHED  0x0080
  #define RS_VELREACHED     0x0100
  #define RS_POSREACHED     0x0200
  #define RS_VZERO          0x0400
  #define RS_ZEROWAIT       0x0800
  #define RS_SECONDMOVE     0x1000
  #define RS_SG             0x2000
