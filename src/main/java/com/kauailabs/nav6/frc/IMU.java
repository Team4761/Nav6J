/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs 2013. All Rights Reserved.                        */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Thunderchicken!           */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. Any        */
/* modifications to this code must be accompanied by the nav6_License.txt file*/ 
/* in the root directory of the project.                                      */
/*----------------------------------------------------------------------------*/

package com.kauailabs.nav6.frc;

import java.util.Arrays;

import com.kauailabs.nav6.IMUProtocol;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * The IMU class provides a simplified interface to the KauaiLabs nav6 IMU.
 * 
 * The IMU class enables access to basic connectivity and state information, 
 * as well as key orientation information (yaw, pitch, roll, compass heading).
 * 
 * Advanced capabilities of the nav6 IMU may be accessed via the IMUAdvanced 
 * class.
 * @author Scott
 */
public class IMU extends SensorBase implements PIDSource, LiveWindowSendable, Runnable {

    static final int    YAW_HISTORY_LENGTH      = 10;
    static final byte   DEFAULT_UPDATE_RATE_HZ  = 100;
    static final short  DEFAULT_ACCEL_FSR_G     = 2;
    static final short  DEFAULT_GYRO_FSR_DPS    = 2000;
    
    SerialPort serialPort;
    float yawHistory[];
    int nextYawHistoryIndex;
    double userYawOffset;
    ITable m_table;
    Thread m_thread;
    protected byte updateRateHz;

    volatile float yaw;
    volatile float pitch;
    volatile float roll;
    volatile float compassHeading;
    volatile int updateCount = 0;
    volatile int byteCount = 0;
    volatile float nav6YawOffsetDegrees;
    volatile short accelFsrG;
    volatile short gyroFsrDps;
    volatile short flags;    

    double lastUpdateTime;
    boolean stop = false;
    private IMUProtocol.YPRUpdate yprUpdateData;
    protected byte updateType = IMUProtocol.MSGID_YPR_UPDATE;
    
    /**
     * Constructs the IMU class, overriding the default update rate
     * with a custom rate which may be from 4 to 100, representing
     * the number of updates per second sent by the nav6 IMU.  
     * 
     * Note that increasing the update rate may increase the 
     * CPU utilization.
     * @param serialPort BufferingSerialPort object to use
     * @param updateRateHz Custom Update Rate (Hz)
     */
    public IMU(SerialPort serialPort, byte updateRateHz) {
        yprUpdateData = new IMUProtocol.YPRUpdate();
        this.updateRateHz = updateRateHz;
        flags = 0;
        accelFsrG = DEFAULT_ACCEL_FSR_G;
        gyroFsrDps = DEFAULT_GYRO_FSR_DPS;
        this.serialPort = serialPort;
        yawHistory = new float[YAW_HISTORY_LENGTH];
        yaw = (float) 0.0;
        pitch = (float) 0.0;
        roll = (float) 0.0;
        try {
            serialPort.reset();
        } catch (RuntimeException ex) {
            ex.printStackTrace();
        }
        initIMU();
        m_thread = new Thread(this);
        m_thread.start();        
    }
    
    /**
     * Constructs the IMU class, using the default update rate.  
     * 
     * @param serial_port BufferingSerialPort object to use
     */
    public IMU(SerialPort serial_port) {
        this(serial_port,DEFAULT_UPDATE_RATE_HZ);
    }

    protected void initIMU() {
        
        // The nav6 IMU serial port configuration is 8 data bits, no parity, one stop bit. 
        // No flow control is used.
        // Conveniently, these are the defaults used by the WPILib's SerialPort class.
        //
        // In addition, the WPILib's SerialPort class also defaults to:
        //
        // Timeout period of 5 seconds
        // Termination ('\n' character)
        // Transmit immediately

        initializeYawHistory();
        userYawOffset = 0;

        // set the nav6 into the desired update mode
        byte streamCommandBuffer[] = new byte[256];
        int packetLength = IMUProtocol.encodeStreamCommand( streamCommandBuffer, updateType, updateRateHz ); 
        try {
            serialPort.write( streamCommandBuffer, packetLength );
        } catch (RuntimeException ex) {
        	ex.printStackTrace();
        }
    }

    protected void setStreamResponse( IMUProtocol.StreamResponse response ) {
        flags = response.flags;
        nav6YawOffsetDegrees = response.yawOffsetDegrees;
        accelFsrG = response.accelFsrG;
        gyroFsrDps = response.gyroFsrDps;
        updateRateHz = (byte)response.updateRateHz;
    }
        
    private void initializeYawHistory() {
        Arrays.fill(yawHistory,0);
        nextYawHistoryIndex = 0;
        lastUpdateTime = 0.0;
    }

    private void setYawPitchRoll(float yaw, float pitch, float roll, float compassHeading) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.compassHeading = compassHeading;

        updateYawHistory(this.yaw);
    }

    protected void updateYawHistory(float currYaw) {
        if (nextYawHistoryIndex >= YAW_HISTORY_LENGTH) {
            nextYawHistoryIndex = 0;
        }
        
        yawHistory[nextYawHistoryIndex] = currYaw;
        lastUpdateTime = Timer.getFPGATimestamp();
        nextYawHistoryIndex++;
    }

    private double getAverageFromYawHistory() {
        double yawHistorySum = 0.0;
        for (int i = 0; i < YAW_HISTORY_LENGTH; i++) {
            yawHistorySum += yawHistory[i];
        }
        
        double yawHistoryAvg = yawHistorySum / YAW_HISTORY_LENGTH;
        return yawHistoryAvg;
    }

    /**
     * Returns the current pitch value (in degrees, from -180 to 180)
     * reported by the nav6 IMU.
     * @return The current pitch value in degrees (-180 to 180).
     */
    public float getPitch() {
        return pitch;
    }

    /**
     * Returns the current roll value (in degrees, from -180 to 180)
     * reported by the nav6 IMU.
     * @return The current roll value in degrees (-180 to 180).
     */
    public float getRoll() {
        return roll;
    }

    /**
     * Returns the current yaw value (in degrees, from -180 to 180)
     * reported by the nav6 IMU.
     * 
     * Note that the returned yaw value will be offset by a user-specified
     * offset value; this user-specified offset value is set by 
     * invoking the zeroYaw() method.
     * @return The current yaw value in degrees (-180 to 180).
     */
    public float getYaw() {
        float calculatedYaw = (float) (this.yaw - userYawOffset);
        if (calculatedYaw < -180) {
            calculatedYaw += 360;
        }
        
        if (calculatedYaw > 180) {
            calculatedYaw -= 360;
        }
        
        return calculatedYaw;
    }

    /**
     * Returns the current tilt-compensated compass heading 
     * value (in degrees, from 0 to 360) reported by the nav6 IMU.
     * 
     * Note that this value is sensed by the nav6 magnetometer,
     * which can be affected by nearby magnetic fields (e.g., the
     * magnetic fields generated by nearby motors).
     * @return The current tilt-compensated compass heading, in degrees (0-360).
     */
    public float getCompassHeading() {
        return compassHeading;
    }

    /**
     * Sets the user-specified yaw offset to the current
     * yaw value reported by the nav6 IMU.
     * 
     * This user-specified yaw offset is automatically
     * subtracted from subsequent yaw values reported by
     * the getYaw() method.
     */
    public void zeroYaw() {
        userYawOffset = getAverageFromYawHistory();
    }

    /**
     * Indicates whether the nav6 IMU is currently connected
     * to the host computer.  A connection is considered established
     * whenever a value update packet has been received from the
     * nav6 IMU within the last second.
     * @return Returns true if a valid update has been received within the last second.
     */
    public boolean isConnected() {
        double timeSinceLastUpdate = Timer.getFPGATimestamp() - this.lastUpdateTime;
        return timeSinceLastUpdate <= 1.0;
    }

    /**
     * Returns the count in bytes of data received from the
     * nav6 IMU.  This could can be useful for diagnosing 
     * connectivity issues.
     * 
     * If the byte count is increasing, but the update count
     * (see getUpdateCount()) is not, this indicates a software
     * misconfiguration.
     * @return The number of bytes received from the nav6 IMU.
     */
    public double getByteCount() {
        return byteCount;
    }

    /**
     * Returns the count of valid update packets which have
     * been received from the nav6 IMU.  This count should increase
     * at the same rate indicated by the configured update rate.
     * @return The number of valid updates received from the nav6 IMU.
     */
    public double getUpdateCount() {
        return updateCount;
    }

    /**
     * Returns true if the nav6 IMU is currently performing automatic
     * calibration.  Automatic calibration occurs when the nav6 IMU
     * is initially powered on, during which time the nav6 IMU should
     * be held still.
     * 
     * During this automatically calibration, the yaw, pitch and roll
     * values returned may not be accurate.
     * 
     * Once complete, the nav6 IMU will automatically remove an internal
     * yaw offset value from all reported values.
     * @return Returns true if the nav6 IMU is currently calibrating.
     */
    public boolean isCalibrating() {
        short calibrationState = (short) (this.flags & IMUProtocol.NAV6_FLAG_MASK_CALIBRATION_STATE);
        return (calibrationState != IMUProtocol.NAV6_CALIBRATION_STATE_COMPLETE);
    }

    /**
     * Returns the current yaw value reported by the nav6 IMU.  This
     * yaw value is useful for implementing features including "auto rotate 
     * to a known angle".
     * @return The current yaw angle in degrees (-180 to 180).
     */
    public double pidGet() {
        return getYaw();
    }

    public void updateTable() {
        if (m_table != null) {
            m_table.putNumber("Value", getYaw());
        }
    }

    public void startLiveWindowMode() {
    }

    public void stopLiveWindowMode() {
    }

    public void initTable(ITable itable) {
        m_table = itable;
        updateTable();
    }

    public ITable getTable() {
        return m_table;
    }

    public String getSmartDashboardType() {
        return "Gyro";
    }

    // Invoked when a new packet is received; returns the packet length if the packet 
    // is valid, based upon IMU Protocol definitions; otherwise, returns 0
    
    protected int decodePacketHandler(byte[] receivedData, int offset, int bytesRemaining) {
        int packetLength = IMUProtocol.decodeYPRUpdate(receivedData, offset, bytesRemaining, yprUpdateData);
        if (packetLength > 0) {
            setYawPitchRoll(yprUpdateData.yaw,yprUpdateData.pitch,yprUpdateData.roll,yprUpdateData.compass_heading);
        }
        
        return packetLength;
    }
    
    // IMU Class thread run method
    
    public void run() {
        stop = false;
        boolean streamResponseReceived = false;
        double lastStreamCommandSentTimestamp = 0.0;
        try {
            serialPort.setReadBufferSize(512);
            serialPort.setTimeout(1.0);
            serialPort.enableTermination('\n');
            serialPort.flush();
            serialPort.reset();
        } catch (RuntimeException ex) {
            ex.printStackTrace();
        }
                
        IMUProtocol.StreamResponse response = new IMUProtocol.StreamResponse();

        byte[] streamCommand = new byte[256];
        
        int cmdPacketLength = IMUProtocol.encodeStreamCommand(streamCommand, updateType, updateRateHz); 
        try {
            serialPort.reset();
            serialPort.write( streamCommand, cmdPacketLength );
            serialPort.flush();
            lastStreamCommandSentTimestamp = Timer.getFPGATimestamp();
        } catch (RuntimeException ex) {
        	ex.printStackTrace();
        }
        
        while (!stop) {
            try {
                // Wait, with delays to conserve CPU resources, until
                // bytes have arrived.
                
                while ( !stop && ( serialPort.getBytesReceived() < 1 ) ) {
                    Timer.delay(0.1);
                }

                int packetsReceived = 0;
                byte[] receivedData = serialPort.read(256);
                int bytesRead = receivedData.length;
                if (bytesRead > 0) {
                    byteCount += bytesRead;
                    int i = 0;
                    // Scan the buffer looking for valid packets
                    while (i < bytesRead) {                    
                        // Attempt to decode a packet
                        int bytesRemaining = bytesRead - i;
                        int packetLength = decodePacketHandler(receivedData,i,bytesRemaining);
                        if (packetLength > 0) {
                            packetsReceived++;
                            updateCount++;
                            i += packetLength;
                        } else {
                            packetLength = IMUProtocol.decodeStreamResponse(receivedData, i, bytesRemaining, response);
                            if (packetLength > 0) {
                                packetsReceived++;
                                setStreamResponse(response);
                                streamResponseReceived = true;
                                i += packetLength;
                            } else {
                                // current index is not the start of a valid packet; increment
                                i++;
                            }
                        }
                    }
                
                    if (packetsReceived == 0 && bytesRead == 256) {
                        // Workaround for issue found in Java SerialPort implementation:
                        // No packets received and 256 bytes received; this
                        // condition occurs in the Java SerialPort.  In this case,
                        // reset the serial port.
                        serialPort.reset();
                    }
                    
                    // If a stream configuration response has not been received within three seconds
                    // of operation, (re)send a stream configuration request
                    
                    if (!streamResponseReceived && ((Timer.getFPGATimestamp() - lastStreamCommandSentTimestamp ) > 3.0)) {
                        cmdPacketLength = IMUProtocol.encodeStreamCommand(streamCommand, updateType, updateRateHz); 
                        try {
                            lastStreamCommandSentTimestamp = Timer.getFPGATimestamp();
                            serialPort.write( streamCommand, cmdPacketLength );
                            serialPort.flush();
                        } catch (RuntimeException ex2) {
                        	ex2.printStackTrace();
                        }                                                    
                    } else {                        
                        // If no bytes remain in the buffer, and not awaiting a response, sleep a bit
                        if ( streamResponseReceived && serialPort.getBytesReceived() == 0) {
                            Timer.delay(1.0/updateRateHz);
                        }        
                    }
                }
            } catch (RuntimeException ex) {
                // This exception typically indicates a Timeout
                streamResponseReceived = false;
                ex.printStackTrace();
            }
        }
    }

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}
}
