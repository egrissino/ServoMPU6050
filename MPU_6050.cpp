/***************************************************************************************************/
/*                                                                                                 */
/* file:          MPU_6050.cpp                                                                     */
/*                                                                                                 */
/* source:        2015-2106, written by Adrian Kundert                                             */
/*                                                                                                 */
/* description:   IMU sensor MPU-6050 I2C device class                                             */
/*                Based on the I2Cdev library collection by Jeff Rowberg <jeff@rowberg.net>        */
/*                and Gyro_header lib from Omer Ikram ul Haq (http://hobbylogs.me.pn/?p=47)        */
/***************************************************************************************************/

#include "MPU_6050.h"

// Defining constants
#define dt 20                        // time difference in milli seconds
#define rad2degree 57.3              // Radian to degree conversion
#define Filter_gain 0.95             // e.g.  angle = angle_gyro*Filter_gain + angle_accel*(1-Filter_gain)

// ----- Configuration ---------------------------------
#define MPU6050_config 26            // R/W
#define MPU6050_gyro_config 27       // R/W
#define MPU6050_accel_config 28      // R/W

// ----- Data ---------------------------------------------
#define MPU6050_data_start 59

// ----- Defining Constant --------------------------------
#define g 9.81                       // Gravitational acceleration

MPU_6050::MPU_6050(uint8_t addr) {
    devAddr = addr;
    dmpReady = false;
    AccelOffset.x = AccelOffset.y = AccelOffset.z = 0;
    GyroOffset.x = GyroOffset.y = GyroOffset.z = 0;
    AccelScaledValue.x = AccelScaledValue.y = AccelScaledValue.z = 0;
    GyroScaledValue.x = GyroScaledValue.y = GyroScaledValue.z = 0;
    AccelScaledFactor = GyroScaledFactor = 0;
    AccelRaw.x = AccelRaw.y = AccelRaw.z = 0;
    GyroRaw.x = GyroRaw.y = GyroRaw.z = 0;
    dataReady = false;
}

void MPU_6050::initialize(bool useDMP, bool doCalibration) {
    DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
    resetAndWake();
    
    if (useDMP == true) {
        // initialize device
        Serial.println(F("Initializing I2C devices..."));
        setClockSource(MPU6050_CLOCK_PLL_XGYRO);
        setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        setSleepEnabled(false);

        // verify connection
        Serial.println("Testing device connections...");
        Serial.println(testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = dmpInitialize();

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            setDMPEnabled(true);

            mpuIntStatus = getIntStatus();

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready! Waiting for first interrupt..."));
            dmpReady = true;

            // get expected DMP packet size for later comparison
            //packetSize = mpu.dmpGetFIFOPacketSize();
        }
        else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
            while (1) {}; // don't try to do anything
        }
    }
    // override some configuration methods of the MPU6050        
    setGains(0, 1); // Setting the lows scale
    if(doCalibration == true) {
        setDLPF(0); // Setting the DLPF to inf Bandwidth for calibration
        offsetCalibration(); // uses default values when not called
    }
    setDLPF(6); // Setting the DLPF to lowest Bandwidth for the application
}

uint8_t MPU_6050::dmpInitialize() {
    // reset device
    DEBUG_PRINTLN(F("\n\nResetting MPU6050..."));
    reset();
    delay(30); // wait after reset

    // enable sleep mode and wake cycle
    /*Serial.println(F("Enabling sleep mode..."));
    setSleepEnabled(true);
    Serial.println(F("Enabling wake cycle..."));
    setWakeCycleEnabled(true);*/

    // disable sleep mode
    DEBUG_PRINTLN(F("Disabling sleep mode..."));
    setSleepEnabled(false);

    // get MPU hardware revision
    DEBUG_PRINTLN(F("Selecting user bank 16..."));
    setMemoryBank(0x10, true, true);
    DEBUG_PRINTLN(F("Selecting memory byte 6..."));
    setMemoryStartAddress(0x06);
    DEBUG_PRINTLN(F("Checking hardware revision..."));
    uint8_t hwRevision = readMemoryByte();
    DEBUG_PRINT(F("Revision @ user[16][6] = "));
    DEBUG_PRINTLNF(hwRevision, HEX);
    DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
    setMemoryBank(0, false, false);

    // check OTP bank valid
    DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
    uint8_t otpValid = getOTPBankValid();
    DEBUG_PRINT(F("OTP bank is "));
    DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    DEBUG_PRINTLN(F("Reading gyro offset TC values..."));
    int8_t xgOffsetTC = getXGyroOffsetTC();
    int8_t ygOffsetTC = getYGyroOffsetTC();
    int8_t zgOffsetTC = getZGyroOffsetTC();
    DEBUG_PRINT(F("X gyro offset = "));
    DEBUG_PRINTLN(xgOffset);
    DEBUG_PRINT(F("Y gyro offset = "));
    DEBUG_PRINTLN(ygOffset);
    DEBUG_PRINT(F("Z gyro offset = "));
    DEBUG_PRINTLN(zgOffset);

    // setup weird slave stuff (?)
    DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
    setSlaveAddress(0, 0x7F);
    DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
    setI2CMasterModeEnabled(false);
    DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
    setSlaveAddress(0, 0x68);
    DEBUG_PRINTLN(F("Resetting I2C Master control..."));
    resetI2CMaster();
    delay(20);

    // load DMP code into memory banks
    DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
    DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
    DEBUG_PRINTLN(F(" bytes)"));
    if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) {
        DEBUG_PRINTLN(F("Success! DMP code written and verified."));

        // write DMP configuration
        DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
        DEBUG_PRINT(MPU6050_DMP_CONFIG_SIZE);
        DEBUG_PRINTLN(F(" bytes in config def)"));
        if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
            DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

            DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
            setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

            DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
            setIntEnabled(0x12);

            DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
            //setRate(4); // 1khz / (1 + 4) = 200 Hz  -> 100isr/sec
            setRate(9); // 1khz / (1 + 9) = 100 Hz  -> 50isr/sec
            //setRate(49); // 1khz / (1 + 49) = 20 Hz  -> 10isr/sec

            DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
            setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

            DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
            setDLPFMode(MPU6050_DLPF_BW_42);

            DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
            setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

            DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
            setOTPBankValid(false);

            DEBUG_PRINTLN(F("Setting X/Y/Z gyro offset TCs to previous values..."));
            setXGyroOffsetTC(xgOffsetTC);
            setYGyroOffsetTC(ygOffsetTC);
            setZGyroOffsetTC(zgOffsetTC);

            //DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
            //setXGyroOffset(0);
            //setYGyroOffset(0);
            //setZGyroOffset(0);

            DEBUG_PRINTLN(F("Writing final memory update 1/7 (function unknown)..."));
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Resetting FIFO..."));
            resetFIFO();

            DEBUG_PRINTLN(F("Reading FIFO count..."));
            uint16_t fifoCount = getFIFOCount();
            uint8_t fifoBuffer[128];

            DEBUG_PRINT(F("Current FIFO count="));
            DEBUG_PRINTLN(fifoCount);
            getFIFOBytes(fifoBuffer, fifoCount);

            DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
            setMotionDetectionThreshold(2);

            DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
            setZeroMotionDetectionThreshold(156);

            DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
            setMotionDetectionDuration(80);

            DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
            setZeroMotionDetectionDuration(0);

            DEBUG_PRINTLN(F("Resetting FIFO..."));
            resetFIFO();

            DEBUG_PRINTLN(F("Enabling FIFO..."));
            setFIFOEnabled(true);

            DEBUG_PRINTLN(F("Enabling DMP..."));
            setDMPEnabled(true);

            DEBUG_PRINTLN(F("Resetting DMP..."));
            resetDMP();

            DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
            while ((fifoCount = getFIFOCount()) < 3);

            DEBUG_PRINT(F("Current FIFO count="));
            DEBUG_PRINTLN(fifoCount);
            DEBUG_PRINTLN(F("Reading FIFO data..."));
            getFIFOBytes(fifoBuffer, fifoCount);

            DEBUG_PRINTLN(F("Reading interrupt status..."));
            uint8_t mpuIntStatus = getIntStatus();

            DEBUG_PRINT(F("Current interrupt status="));
            DEBUG_PRINTLNF(mpuIntStatus, HEX);

            DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
            while ((fifoCount = getFIFOCount()) < 3);

            DEBUG_PRINT(F("Current FIFO count="));
            DEBUG_PRINTLN(fifoCount);

            DEBUG_PRINTLN(F("Reading FIFO data..."));
            getFIFOBytes(fifoBuffer, fifoCount);

            DEBUG_PRINTLN(F("Reading interrupt status..."));
            mpuIntStatus = getIntStatus();

            DEBUG_PRINT(F("Current interrupt status="));
            DEBUG_PRINTLNF(mpuIntStatus, HEX);

            DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            DEBUG_PRINTLN(F("DMP is good to go! Finally."));

            DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
            setDMPEnabled(false);

            DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
            dmpPacketSize = 42;
            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

            DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
            resetFIFO();
            getIntStatus();
        } else {
            DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
            return 2; // configuration block loading failed
        }
    } else {
        DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
        return 1; // main binary block loading failed
    }
    return 0; // success
}

void MPU_6050::setAccelOffset(int x, int y, int z) {
    AccelOffset.x = x;
    AccelOffset.y = y;
    AccelOffset.z = z;
}

void MPU_6050::setGyroOffset(int x, int y, int z) {
    GyroOffset.x = x;
    GyroOffset.y = y;
    GyroOffset.z = z;
}

// ------------------------------------------------
//       Setting up the DLFP
// ------------------------------------------------
void MPU_6050::setDLPF(int BW) {
    if (BW < 0 || BW > 6){
    BW = 0;
    }
    Wire.beginTransmission(devAddr);
    Wire.write(devAddr); // Address to the configuration register
    /*       config Discription ---- x x 0 0 0 F2 F1 F0
    I am only intrested in the Digital Low Pass Filter (DLPF)
    F2 F1 F0    Bandwidth [Hz]
    0  0  0
    0  0  1      184
    0  1  0      94
    0  1  1      44
    1  0  0      21
    1  0  1      10
    1  1  0      5
    */
    Wire.write(BW);
    Wire.endTransmission();
}

// ------------------------------------------------
//       Setting up the Accelrometer and Gyro Gains
// ------------------------------------------------
void MPU_6050::setGains(int gyro,int accel) {
    byte gyro_byte, accel_byte;

    // Setting up Gyro
    Wire.beginTransmission(devAddr);
    Wire.write(MPU6050_gyro_config); // Address to the configuration register
    if (gyro==0)
    {
      GyroScaledFactor =(float)250/0x7fff; // each data is of 16 bits that means, 250 is divided along 2^(15)-1 = 32767
      gyro_byte = 0b00000000;
    }else if (gyro == 1)
    {
      GyroScaledFactor = (float)500/0x7fff; // each data is of 16 bits that means, 500 is divided along 2^(15)-1 = 32767
      gyro_byte = 0b00001000;
    }else if (gyro == 2)
    {
      GyroScaledFactor = (float)1000/0x7fff;// each data is of 16 bits that means, 1000 is divided along 2^(15)-1 = 32767
      gyro_byte = 0b00010000;
    }else if (gyro == 3)
    {
      GyroScaledFactor = (float)2000/0x7fff;  // each data is of 16 bits that means, 2000 is divided along 2^(15)-1 = 32767
      gyro_byte = 0b00011000;
    }else
    {
      GyroScaledFactor = 1;
    }

    Wire.write(gyro_byte);
    Wire.endTransmission();
    Serial.print("The gyro scale is set to ");
    Serial.print(GyroScaledFactor*1000);
    Serial.println("E-3 Degree/sample");

    // Setting up Accel
    Wire.beginTransmission(devAddr);
    Wire.write(MPU6050_accel_config); // Address to the configuration register
    if (accel==0)
    {
      AccelScaledFactor = (float)2*g/0x7fff; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767
      accel_byte = 0b00000000;
    }else if (accel == 1)
    {
      AccelScaledFactor = (float)4*g/0x7fff; // each data is of 16 bits that means, 4g is divided along 2^(15)-1 = 32767
      accel_byte = 0b00001000;
    }else if (accel == 2)
    {
      AccelScaledFactor = (float)8*g/0x7fff;// each data is of 16 bits that means, 8g is divided along 2^(15)-1 = 32767
      accel_byte = 0b00010000;
    }else if (accel == 3)
    {
      AccelScaledFactor = (float)16*g/0x7fff; // each data is of 16 bits that means, 16g is divided along 2^(15)-1 = 32767
      accel_byte = 0b00011000;
    }else
    {
      AccelScaledFactor = 1;
    }

    Wire.write(accel_byte);
    Wire.endTransmission();
    Serial.print("The accel scale is set to ");
    Serial.print(AccelScaledFactor*1000);
    Serial.println("E-3 m/s^2 / sample");
 }

// ------------------------------------------------
//       offset calibration (we assume the break board is lying flat)
// ------------------------------------------------

void MPU_6050::offsetCalibration(){
    Serial.println("Calibrating gyroscope .... dont move the hardware ..........");

    int i,cnt=100;
    long x=0,y=0,z=0;

    // throw first 10 measurements
    for (i=1;i<=10;i++) {
        getDataForCalibration();
    }

    // Gyro Offset Calculation
    for (i=1;i<=cnt;i++) {
        getDataForCalibration();
        x += GyroRaw.x;
        y += GyroRaw.y;
        z += GyroRaw.z;
        Serial.print(".");
    }
    Serial.println(".");
    GyroOffset.x = x/cnt;
    GyroOffset.y = y/cnt;
    GyroOffset.z = z/cnt;

    Serial.print("gyro_x register offset = ");
    Serial.println(GyroOffset.x);

    Serial.print("gyro_y register offect = ");
    Serial.println(GyroOffset.y);

    Serial.print("gyro_z register offset = ");
    Serial.println(GyroOffset.z);

    // Accel Offset Calculation
    Serial.println("Calibrating accelerometer .... dont move the hardware ..........");
    x=y=z=0;
    // throw first 10 measurements
    for (i=1;i<=10;i++) {
        getDataForCalibration();
    }
    for (i=1;i<=cnt;i++) {
        getDataForCalibration();
        x += AccelRaw.x;
        y += AccelRaw.y;
        z += AccelRaw.z;
        Serial.print(".");
    }
    Serial.println(".");
    AccelOffset.x = x/cnt;
    AccelOffset.y = y/cnt;
    AccelOffset.z = z/cnt -(float)g/AccelScaledFactor; // include the 1g in the offset calc

    Serial.print("Accel_x register offset = ");
    Serial.println(AccelOffset.x);

    Serial.print("Accel_y register offect = ");
    Serial.println(AccelOffset.y);

    Serial.print("Accel_z register offset = ");
    Serial.println(AccelOffset.z);
}
void MPU_6050::getDataForCalibration() {
    while(dataReady == false) {
        if (dmpReady == false) {
            delay(50); // the MPU period is according to the rate: should be 50ms.
            getRawData(); //poll instead interrupt usage
        }
    };  // wait interrupt
    dataReady = false;
}

void MPU_6050::getRawData() {
  AccelRaw.x = getAccelerationX();
  AccelRaw.y = getAccelerationY();
  AccelRaw.z = getAccelerationZ();

  GyroRaw.x = getRotationX();
  GyroRaw.y = getRotationY();
  GyroRaw.z = getRotationZ();
  dataReady = true;  // data transfered from MPU
}

void MPU_6050::processData() {

    AccelScaledValue.x = (float)(AccelRaw.x-AccelOffset.x)*AccelScaledFactor;
    AccelScaledValue.y = (float)(AccelRaw.y-AccelOffset.y)*AccelScaledFactor;
    AccelScaledValue.z = (float)(AccelRaw.z-AccelOffset.z)*AccelScaledFactor;

    GyroScaledValue.x = (float)(GyroRaw.x-GyroOffset.x)*GyroScaledFactor;
    GyroScaledValue.y = (float)(GyroRaw.y-GyroOffset.y)*GyroScaledFactor;
    GyroScaledValue.z = (float)(GyroRaw.z-GyroOffset.z)*GyroScaledFactor;
    dataReady = false;  // data consummed, ready for another transfer

    // angle calculation: 1*(90-abs(Angle.x))/90 nulls the integration when the rotation is vertical, a gain of 1 is used
    float xdt = (90-abs(Angle.x))/90 * GyroScaledValue.x * ((float)dt / 1000); // x rotation integration gives y and z angle displacement
    float ydt = (90-abs(Angle.y))/90 * GyroScaledValue.y * ((float)dt / 1000); // y rotation integration gives x and z angle displacement
    float zdt = (90-abs(Angle.z))/90 * GyroScaledValue.z * ((float)dt / 1000); // z rotation integration gives x and y angle displacement
    AngleGyro.x = Angle.x + zdt - ydt; 
    AngleGyro.y = Angle.y + xdt - zdt; 
    AngleGyro.z = Angle.z + ydt - xdt;

    AngleAccel.x = atan(AccelScaledValue.x / (sqrt(AccelScaledValue.y * AccelScaledValue.y + AccelScaledValue.z * AccelScaledValue.z))) * (float)rad2degree;
    AngleAccel.y = atan(AccelScaledValue.y / (sqrt(AccelScaledValue.z * AccelScaledValue.z + AccelScaledValue.x * AccelScaledValue.x))) * (float)rad2degree;
    AngleAccel.z = atan(AccelScaledValue.z / (sqrt(AccelScaledValue.y * AccelScaledValue.y + AccelScaledValue.x * AccelScaledValue.x))) * (float)rad2degree;

    Angle.x = Filter_gain * AngleGyro.x + (1 - Filter_gain) * AngleAccel.x;
    Angle.y = Filter_gain * AngleGyro.y + (1 - Filter_gain) * AngleAccel.y;
    Angle.z = Filter_gain * AngleGyro.z + (1 - Filter_gain) * AngleAccel.z;
}

void MPU_6050::resetFIFO() {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}
void MPU_6050::setIntEnabled(uint8_t enabled) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, enabled);
}
void MPU_6050::setRate(uint8_t rate) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}
void MPU_6050::setExternalFrameSync(uint8_t sync) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}
uint8_t MPU_6050::getDLPFMode() {
  I2Cdev::readBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, buffer);
  return buffer[0];
}
void MPU_6050::setDLPFMode(uint8_t mode) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}
void MPU_6050::setDMPConfig1(uint8_t config) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_1, config);
}
void MPU_6050::setDMPConfig2(uint8_t config) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_2, config);
}
void MPU_6050::setSlaveAddress(uint8_t num, uint8_t address) {
  if (num > 3) return;
  I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}
void MPU_6050::setI2CMasterModeEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
void MPU_6050::resetI2CMaster() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
}
uint8_t MPU_6050::getOTPBankValid() {
    I2Cdev::readBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer);
    return buffer[0];
}
void MPU_6050::setOTPBankValid(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t MPU_6050::getXGyroOffsetTC() {
    I2Cdev::readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU_6050::setXGyroOffsetTC(int8_t offset) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
int8_t MPU_6050::getYGyroOffsetTC() {
    I2Cdev::readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU_6050::setYGyroOffsetTC(int8_t offset) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
int8_t MPU_6050::getZGyroOffsetTC() {
    I2Cdev::readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU_6050::setZGyroOffsetTC(int8_t offset) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
uint16_t MPU_6050::getFIFOCount() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
  return (((uint16_t)buffer[0]) << 8) | buffer[1];
}
void MPU_6050::getFIFOBytes(uint8_t *data, uint8_t length) {
  I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data);
}
void MPU_6050::setMotionDetectionThreshold(uint8_t threshold) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_THR, threshold);
}
void MPU_6050::setZeroMotionDetectionThreshold(uint8_t threshold) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}
void MPU_6050::setMotionDetectionDuration(uint8_t duration) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_DUR, duration);
}
void MPU_6050::setZeroMotionDetectionDuration(uint8_t duration) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}
void MPU_6050::setFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}
void MPU_6050::setDMPEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
void MPU_6050::resetDMP() {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}
uint8_t MPU_6050::getIntStatus() {
  I2Cdev::readByte(devAddr, MPU6050_RA_INT_STATUS, buffer);
  return buffer[0];
}
void MPU_6050::setClockSource(uint8_t source) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}
void MPU_6050::setFullScaleGyroRange(uint8_t range) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}
void MPU_6050::setFullScaleAccelRange(uint8_t range) {
  I2Cdev::writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
void MPU_6050::reset() {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}
void MPU_6050::resetAndWake() {
  Wire.beginTransmission(devAddr);
  Wire.write(MPU6050_RA_PWR_MGMT_1);
  Wire.write(0b10000000);
  Wire.endTransmission();
  
  delay(100); // Waiting for the reset to complete
   
  Wire.beginTransmission(devAddr);
  Wire.write(MPU6050_RA_PWR_MGMT_1);
 
  Wire.write(0b00000000);
  Wire.endTransmission();  
}
void MPU_6050::setSleepEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
void MPU_6050::setWakeCycleEnabled(bool enabled) {
  I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}
int16_t MPU_6050::getAccelerationX() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU_6050::getAccelerationY() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_YOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU_6050::getAccelerationZ() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_ZOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU_6050::getRotationX() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU_6050::getRotationY() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_YOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
int16_t MPU_6050::getRotationZ() {
  I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}
uint8_t MPU_6050::getDeviceID() {
  I2Cdev::readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
  return buffer[0];
}
bool MPU_6050::testConnection() {
  return getDeviceID() == 0x34;
}
uint8_t MPU_6050::readMemoryByte() {
  I2Cdev::readByte(devAddr, MPU6050_RA_MEM_R_W, buffer);
  return buffer[0];
}
void MPU_6050::setMemoryStartAddress(uint8_t address) {
  I2Cdev::writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
}

void MPU_6050::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
  bank &= 0x1F;
  if (userBank) bank |= 0x20;
  if (prefetchEnabled) bank |= 0x40;
  I2Cdev::writeByte(devAddr, MPU6050_RA_BANK_SEL, bank);
}

void MPU_6050::readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
  setMemoryBank(bank);
  setMemoryStartAddress(address);
  uint8_t chunkSize;
  for (uint16_t i = 0; i < dataSize;) {
      // determine correct chunk size according to bank position and data size
      chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

      // make sure we don't go past the data size
      if (i + chunkSize > dataSize) chunkSize = dataSize - i;

      // make sure this chunk doesn't go past the bank boundary (256 bytes)
      if (chunkSize > 256 - address) chunkSize = 256 - address;

      // read the chunk of data as specified
      I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, data + i);

      // increase byte index by [chunkSize]
      i += chunkSize;

      // uint8_t automatically wraps to 0 at 256
      address += chunkSize;

      // if we aren't done, update bank (if necessary) and address
      if (i < dataSize) {
          if (address == 0) bank++;
          setMemoryBank(bank);
          setMemoryStartAddress(address);
      }
  }
}

bool MPU_6050::writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem) {
    uint8_t *progBuffer, success, special;
    uint16_t i, j;
    if (useProgMem) {
        progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
        if (useProgMem) {
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        } else {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            /*Serial.print("Writing config block to bank ");
            Serial.print(bank);
            Serial.print(", offset ");
            Serial.print(offset);
            Serial.print(", length=");
            Serial.println(length);*/
            if (useProgMem) {
                if (sizeof(progBuffer) < length) progBuffer = (uint8_t *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
            } else {
                progBuffer = (uint8_t *)data + i;
            }
            success = writeMemoryBlock(progBuffer, length, bank, offset, true);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = pgm_read_byte(data + i++);
            } else {
                special = data[i++];
            }
            /*Serial.print("Special command code ");
            Serial.print(special, HEX);
            Serial.println(" found...");*/
            if (special == 0x01) {
                // enable DMP-related interrupts

                //setIntZeroMotionEnabled(true);
                //setIntFIFOBufferOverflowEnabled(true);
                //setIntDMPEnabled(true);
                I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = true;
            } else {
                // unknown special command
                success = false;
            }
        }

        if (!success) {
            if (useProgMem) free(progBuffer);
            return false; // uh oh
        }
    }
    if (useProgMem) free(progBuffer);
    return true;
}

bool MPU_6050::writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
  return writeDMPConfigurationSet(data, dataSize, true);
}

bool MPU_6050::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
  return writeMemoryBlock(data, dataSize, bank, address, verify, true);
}

bool MPU_6050::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem) {
    setMemoryBank(bank);
    setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer;
    uint8_t *progBuffer;
    uint16_t i;
    uint8_t j;
    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        I2Cdev::writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        // verify data if needed
        if (verify && verifyBuffer) {
            setMemoryBank(bank);
            setMemoryStartAddress(address);
            I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                /*Serial.print("Block write verification error, bank ");
                Serial.print(bank, DEC);
                Serial.print(", address ");
                Serial.print(address, DEC);
                Serial.print("!\nExpected:");
                for (j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (progBuffer[j] < 16) Serial.print("0");
                    Serial.print(progBuffer[j], HEX);
                }
                Serial.print("\nReceived:");
                for (uint8_t j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (verifyBuffer[i + j] < 16) Serial.print("0");
                    Serial.print(verifyBuffer[i + j], HEX);
                }
                Serial.print("\n");*/
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(bank);
            setMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return true;
}



