package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {
  public final static int FIRMWARE_REV_REGISTER = 0x00;
  public final static int MANUFACTURER_REGISTER = 0x01;
  public final static int SENSOR_ID_REGISTER = 0x02;
  public final static int COMMAND_REGISTER = 0x03;
  public final static int COLOR_NUMBER_REGISTER = 0x04;
  public final static int RED_VALUE_REGISTER = 0x05;
  public final static int GREEN_VALUE_REGISTER = 0x06;
  public final static int BLUE_VALUE_REGISTER = 0x07;
  public final static int WHITE_VALUE_REGISTER = 0x08;
  public final static int COLOR_INDEX_REGISTER = 0x09;
  public final static int RED_INDEX_REGISTER = 0x0A;
  public final static int GREEN_INDEX_REGISTER = 0x0B;
  public final static int BLUE_INDEX_REGISTER = 0x0C;
  public final static int UNDEFINED_REGISTER = 0x0D;

  public final static int RED_READING_REGISTER = 0x0E;
  public final static int GREEN_READING_REGISTER = 0x10;
  public final static int BLUE_READING_REGISTER = 0x12;
  public final static int WHITE_READING_REGISTER_LSB = 0x14;
  public final static int WHITE_READING_REGISTER_MSB = 0x15;
  public final static int NORM_RED_READ_REGISTER = 0x16;
  public final static int NORM_GREEN_READ_REGISTER = 0x18;
  public final static int NORM_BLUE_READ_REGISTER = 0x1A;
  public final static int NORM_WHITE_READ_REGISTER = 0x1C;

  public final static int ACTIVE_MODE_COMMAND = 0x00;
  public final static int PASSIVE_MODE_COMMAND = 0x01;
  public final static int OP_FREQ_50HZ_COMMAND = 0x35;
  public final static int OP_FREQ_60HZ_COMMAND = 0x36;
  public final static int BLACK_CAL_COMMAND = 0x42;
  public final static int WHITE_BAL_COMMAND = 0x43;

  private final I2C i2c;
  private final byte[] byteBuff = new byte[1];

  public ColorSensor(int address, boolean onBoardPort) {
    if (onBoardPort){
      i2c = new I2C(I2C.Port.kOnboard, address >> 1);
    }else{
      i2c = new I2C(I2C.Port.kMXP, address >> 1);
    }
  }
//omg..... id idnt..m.,. knwo know that/a.a....  
  public byte readByte(int register) {
    boolean success = !i2c.read(register, byteBuff.length, byteBuff);
    //System.out.println("Success: " + success);
    return byteBuff[0];
  }

  private int read16bitInt(int lsbRegister, int msbRegister) {  // for data that takes two 8-bit values
    byte[] buf = new byte[2];
    buf[1] = readByte(lsbRegister);
    buf[0] = readByte(msbRegister);
    ByteBuffer bb = ByteBuffer.wrap(buf);
    return bb.getShort() & 0xffff; // mask unsigned short value
  }

  public void issueCommand(int cmd) {
    i2c.write(ColorSensor.COMMAND_REGISTER, cmd);
  }
}




