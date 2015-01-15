/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 *
 * @author Developer
 */
public class ITG3200_I2C {
    private static final byte kAddress = 0x68;
    private static final byte kPowerManagement = 0x3E;
    private static final byte kPowerControlRegister = 0x00;
    // could not find kdataformatregister
    //private static final byte kDataRegister = 0x21;
    private static final double kGsPerLSB = 14.375;
    
    
    private I2C m_i2c;
    
    public ITG3200_I2C(Port i2cPort) {
        // dataFormat value
        //byte range = 0x00;
        
        m_i2c = new I2C(i2cPort, kAddress);
        
        m_i2c.write(kPowerManagement, kPowerControlRegister);
    }
    
    public double getRate() { // Returns degrees per second.
        byte[] rawTurn = new byte[2];
        m_i2c.read(kAddress, rawTurn.length, rawTurn);
        
        return getRateFromRawBytes(rawTurn[1], rawTurn[0]);
    }
    
    private double getRateFromRawBytes(byte low, byte high) {
        short tempLow = (short) (low & 0xff);
        short tempHigh = (short) ((high << 8) & 0xff00);
        return (tempLow | tempHigh) * kGsPerLSB;
        // this shifts high over 8 bits and then combines it with low
        
    } 
}
