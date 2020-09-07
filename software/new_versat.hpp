#include "versat.h"

//Macro functions to use cpu interface
#define MEMSET(base, location, value) (*((volatile int*) (base + (sizeof(int)) * location)) = value)
#define MEMGET(base, location)        (*((volatile int*) (base + (sizeof(int)) * location)))

//constants
#define RUN (1<<(ADDR_W-2-1-N_SLAVES_W-2-2)) //2 -> picoRV, 1 -> STAGE_W
#define CLEAR (RUN + (1<<(ADDR_W-2-1-N_SLAVES_W-2-2-1)))

//
// VERSAT CLASSES
//

class CYoloRead {
  public:
    int base;

    //Default constructor
    CYoloRead() {
    }

    //Constructor with an associated base
    CYoloRead (int base) {
      this->base = base;
    }

    //Methods to set config parameters
    void setExtAddr(int extAddr) {
      MEMSET(base, XYOLO_READ_CONF_EXT_ADDR, extAddr);
    }
    void setOffset(int offset) {
      MEMSET(base, XYOLO_READ_CONF_OFFSET, offset);
    }
    void setPingPong(int pp) {
      MEMSET(base, XYOLO_READ_CONF_PP, pp);
    }
    void setIntAddr(int intAddr) {
      MEMSET(base, XYOLO_READ_CONF_INT_ADDR, intAddr);
    }
    void setExtIter(int iter) {
      MEMSET(base, XYOLO_READ_CONF_ITER_A, iter);
    }
    void setExtPer(int per) {
      MEMSET(base, XYOLO_READ_CONF_PER_A, per);
    }
    void setExtShift(int shift) {
      MEMSET(base, XYOLO_READ_CONF_SHIFT_A, shift);
    }
    void setExtIncr(int incr) {
      MEMSET(base, XYOLO_READ_CONF_INCR_A, incr);
    }
    void setIntIter(int iter) {
      MEMSET(base, XYOLO_READ_CONF_ITER_B, iter);
    }
    void setIntPer(int per) {
      MEMSET(base, XYOLO_READ_CONF_PER_B, per);
    }
    void setIntStart(int start) {
      MEMSET(base, XYOLO_READ_CONF_START_B, start);
    }
    void setIntShift(int shift) {
      MEMSET(base, XYOLO_READ_CONF_SHIFT_B, shift);
    }
    void setIntIncr(int incr) {
      MEMSET(base, XYOLO_READ_CONF_INCR_B, incr);
    }
    void setBiasExtAddr(int extAddr) {
      MEMSET(base, BIAS_CONF_EXT_ADDR, extAddr);
    }
    void setBiasIntAddr(int intAddr) {
      MEMSET(base, BIAS_CONF_INT_ADDR, intAddr);
    }
    void setBiasIntStart(int start) {
      MEMSET(base, BIAS_CONF_START_B, start);
    }
};//end class CYoloRead

//vread from xyolo_write
class CRead {
  public:
    int base;

    //Default constructor
    CRead() {
    }

    //Constructor with an associated base
    CRead (int base) {
      this->base = base;
    }

    //Methods to set config parameters
    void setExtAddr(int extAddr) {
      MEMSET(base, VREAD_CONF_EXT_ADDR, extAddr);
    }
    void setOffset(int offset) {
      MEMSET(base, VREAD_CONF_OFFSET, offset);
    }
    void setPingPong(int pp) {
      MEMSET(base, VREAD_CONF_PP, pp);
    }
    void setIntAddr(int intAddr) {
      MEMSET(base, VREAD_CONF_INT_ADDR, intAddr);
    }
    void setExtIter(int iter) {
      MEMSET(base, VREAD_CONF_ITER_A, iter);
    }
    void setExtPer(int per) {
      MEMSET(base, VREAD_CONF_PER_A, per);
    }
    void setExtShift(int shift) {
      MEMSET(base, VREAD_CONF_SHIFT_A, shift);
    }
    void setExtIncr(int incr) {
      MEMSET(base, VREAD_CONF_INCR_A, incr);
    }
    void setIntStart(int start) {
      MEMSET(base, VREAD_CONF_START_B, start);
    }
    void setIntIter(int iter) {
      MEMSET(base, VREAD_CONF_ITER_B, iter);
    }
    void setIntPer(int per) {
      MEMSET(base, VREAD_CONF_PER_B, per);
    }
    void setIntShift(int shift) {
      MEMSET(base, VREAD_CONF_SHIFT_B, shift);
    }
    void setIntIncr(int incr) {
      MEMSET(base, VREAD_CONF_INCR_B, incr);
    }
    void setIntIter2(int iter) {
      MEMSET(base, VREAD_CONF_ITER2_B, iter);
    }
    void setIntPer2(int per) {
      MEMSET(base, VREAD_CONF_PER2_B, per);
    }
    void setIntShift2(int shift) {
      MEMSET(base, VREAD_CONF_SHIFT2_B, shift);
    }
    void setIntIncr2(int incr) {
      MEMSET(base, VREAD_CONF_INCR2_B, incr);
    }
    void setIntIter3(int iter) {
      MEMSET(base, VREAD_CONF_ITER3_B, iter);
    }
    void setIntPer3(int per) {
      MEMSET(base, VREAD_CONF_PER3_B, per);
    }
    void setIntShift3(int shift) {
      MEMSET(base, VREAD_CONF_SHIFT3_B, shift);
    }
    void setIntIncr3(int incr) {
      MEMSET(base, VREAD_CONF_INCR3_B, incr);
    }
};//end class CRead

//vwrite from xyolo_write
class CWrite {
  public:
    int base;

    //Default constructor
    CWrite() {
    }

    //Constructor with an associated base
    CWrite (int base) {
      this->base = base;
    }

    //Methods to set config parameters
    void setExtAddr(int extAddr) {
      MEMSET(base, VWRITE_CONF_EXT_ADDR, extAddr);
    }
    void setOffset(int offset) {
      MEMSET(base, VWRITE_CONF_OFFSET, offset);
    }
    void setIntAddr(int intAddr) {
      MEMSET(base, VWRITE_CONF_INT_ADDR, intAddr);
    }
    void setExtIter(int iter) {
      MEMSET(base, VWRITE_CONF_ITER_A, iter);
    }
    void setExtPer(int per) {
      MEMSET(base, VWRITE_CONF_PER_A, per);
    }
    void setExtShift(int shift) {
      MEMSET(base, VWRITE_CONF_SHIFT_A, shift);
    }
    void setExtIncr(int incr) {
      MEMSET(base, VWRITE_CONF_INCR_A, incr);
    }
    void setIntStart(int start) {
      MEMSET(base, VWRITE_CONF_START_B, start);
    }
    void setIntDuty(int duty) {
      MEMSET(base, VWRITE_CONF_DUTY_B, duty);
    }
    void setIntDelay(int delay) {
      MEMSET(base, VWRITE_CONF_DELAY_B, delay);
    }
    void setIntIter(int iter) {
      MEMSET(base, VWRITE_CONF_ITER_B, iter);
    }
    void setIntPer(int per) {
      MEMSET(base, VWRITE_CONF_PER_B, per);
    }
    void setIntShift(int shift) {
      MEMSET(base, VWRITE_CONF_SHIFT_B, shift);
    }
    void setIntIncr(int incr) {
      MEMSET(base, VWRITE_CONF_INCR_B, incr);
    }
};//end class CWrite

//xyolo from xyolo_write
class CYolo {
  public:
    int base;

    //Default constructor
    CYolo() {
    }

    //Constructor with an associated base
    CYolo(int base) {
      this->base = base;
    }

    //Methods to set config parameters
    void setIter(int iter) {
      MEMSET(base, XYOLO_CONF_ITER, iter);
    }
    void setPer(int per) {
      MEMSET(base, XYOLO_CONF_PER, per);
    }
    void setShift(int shift) {
      MEMSET(base, XYOLO_CONF_SHIFT, shift);
    }
    void setBias(int bias) {
      MEMSET(base, XYOLO_CONF_BIAS, bias);
    }
    void setLeaky(int leaky) {
      MEMSET(base, XYOLO_CONF_LEAKY, leaky);
    }
    void setSigmoid(int sigmoid) {
      MEMSET(base, XYOLO_CONF_SIGMOID, sigmoid);
    }
    void setSigMask(int mask) {
      MEMSET(base, XYOLO_CONF_SIG_MASK, mask);
    }
    void setMaxpool(int maxpool) {
      MEMSET(base, XYOLO_CONF_MAXPOOL, maxpool);
    }
    void setBypass(int bypass) {
      MEMSET(base, XYOLO_CONF_BYPASS, bypass);
    }
};//end class CYOLO

class CYoloWrite {

  public:
    CRead read;
    CYolo yolo;
    CWrite write;

    //Default constructor
    CYoloWrite() {
    }

    //Constructor with an associated base
    CYoloWrite(int base) {

      //Init functional units
      read = CRead(base);
      yolo = CYolo(base);
      write = CWrite(base);
    }
};//end class CYoloWrite

class CDma {

  public:
    int base;
  
    //Default constructor
    CDma(){
    }

    //Constructor with an associated base
    CDma(int base) {
      this->base = base;
    }

    //Methods to set config parameters
    void yread_setLen(int len) {
      MEMSET(base, DMA_XYOLO_READ_CONF_LEN, len);
    }
    void ywrite_read_setLen(int len) {
      MEMSET(base, DMA_XYOLO_WRITE_READ_CONF_LEN, len);
    }
    void ywrite_write_setLen(int len) {
      MEMSET(base, DMA_XYOLO_WRITE_WRITE_CONF_LEN, len);
    }  
};//end class CDma
  
class CVersat {

  public:
    int versat_base;
    CYoloRead yread;
    CYoloWrite ywrite;
    CDma dma;

    //Default constructor
    CVersat() {
    }

    CVersat(int base) {

      //Store versat base
      this->versat_base = base;

      //Init stages
      yread = CYoloRead(base);
      ywrite = CYoloWrite(base + (1<<(ADDR_W-2-1-N_SLAVES_W-1)));
      dma = CDma(base + (1<<(ADDR_W-2-1-N_SLAVES_W)));
    }

    //Methods
    void run() {
      MEMGET(versat_base, RUN); //could be either GET or SET
    }
    int done() {
      return MEMGET(versat_base, 0); //versat always returns done value regardless of address
    }
    void clear() {
      MEMGET(versat_base, CLEAR); //could be either GET or SET
    }
};//end class CVersat

//Global object
CVersat versat;

//
//VERSAT FUNCTIONS
//

inline void versat_init(int base_addr) {

  //init versat object
  versat = CVersat(base_addr);
}

inline void versat_end() {
  //last 2 runs
  while(versat.done()==0);
  versat.run();
  while(versat.done()==0);
  versat.run();
  while(versat.done()==0);
}
