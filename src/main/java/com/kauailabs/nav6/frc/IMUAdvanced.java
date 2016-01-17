/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs 2013. All Rights Reserved.                       */
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

import edu.wpi.first.wpilibj.SerialPort;

/**
 * The IMUAdvanced class provides a simplified interface to advanced capabilities
 * of the KauaiLabs nav6 IMU.
 * 
 * The IMUAdvanced class enables access to basic connectivity and state information, 
 * as well as key orientation information (yaw, pitch, roll, compass heading).
 * Additionally, the IMUAdvanced class also provides access to extended information
 * including linear acceleration, motion detection, and sensor temperature.
 * @author Scott
 */public class IMUAdvanced extends IMU {

    private IMUProtocol.QuaternionUpdate quaternionUpdateData;    
    volatile float worldLinearAccelX;
    volatile float worldLinearAccelY;
    volatile float worldLinearAccelZ;
    volatile float tempC;
    float worldLinearAccelHistory[];
    int   nextWorldLinearAccelHistoryIndex;
    float worldLinearAccelerationRecentAvg;
    
    static final int WORLD_LINEAR_ACCEL_HISTORY_LENGTH = 10;

    /**
     * Constructs the IMUAdvanced class, overriding the default update rate
     * with a custom rate which may be from 4 to 100, representing
     * the number of updates per second sent by the nav6 IMU.  
     * 
     * Note that increasing the update rate may increase the 
     * CPU utilization.  Note that calculation of some 
     * advanced values utilizes additional cpu cycles, when compared
     * to the IMU class.
     * @param serialPort BufferingSerialPort object to use
     * @param updateRateHz Custom Update Rate (Hz)
     */
    public IMUAdvanced(SerialPort serialPort, byte updateRateHz) {
        super(serialPort, updateRateHz);
        quaternionUpdateData = new IMUProtocol.QuaternionUpdate();
        updateType = IMUProtocol.MSGID_QUATERNION_UPDATE;
    }
    
    /**
     * Constructs the IMUAdvanced class, using the default update rate.  
     * 
     * Note that calculation of some advanced values utilizes additional 
     * cpu cycles, when compared to the IMU class.
     * @param serialPort BufferingSerialPort object to use
     */
    public IMUAdvanced(SerialPort serialPort) {
        this(serialPort, DEFAULT_UPDATE_RATE_HZ);
    }

    //@Override
    protected int decodePacketHandler(byte[] receivedData, int offset, int bytesRemaining) {
        
        int packetLength = IMUProtocol.decodeQuaternionUpdate(receivedData, offset, bytesRemaining, quaternionUpdateData);
        if (packetLength > 0) {
            setQuaternion(quaternionUpdateData);
        }
        return packetLength;
    }
        
    /**
     * Returns the current linear acceleration in the x-axis (in g).
     * 
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the x-axis of the
     * body (e.g., the robot) on which the nav6 IMU is mounted.
     * 
     * @return Current world linear acceleration in the x-axis (in g).
     */
    public float getWorldLinearAccelX()
    {
        return this.worldLinearAccelX;
    }

    /**
     * Returns the current linear acceleration in the y-axis (in g).
     * 
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the y-axis of the
     * body (e.g., the robot) on which the nav6 IMU is mounted.
     * 
     * @return Current world linear acceleration in the y-axis (in g).
     */
    public float getWorldLinearAccelY()
    {
        return this.worldLinearAccelY;
    }

    /**
     * Returns the current linear acceleration in the z-axis (in g).
     * 
     * World linear acceleration refers to raw acceleration data, which
     * has had the gravity component removed, and which has been rotated to
     * the same reference frame as the current yaw value.  The resulting
     * value represents the current acceleration in the z-axis of the
     * body (e.g., the robot) on which the nav6 IMU is mounted.
     * 
     * @return Current world linear acceleration in the z-axis (in g).
     */
    public float getWorldLinearAccelZ()
    {
        return this.worldLinearAccelZ;
    }
    
    /**
     * Gets the angle returned from the gyro without constraining it
     * to a certain range
     * @return the angle from -infinity to infinity
     */
    public float getAccumulatedYaw() {
    	return (float) (this.yaw - user_yaw_offset);
    }

    /**
     * Indicates if the nav6 IMU is currently detection motion,
     * based upon the x and y-axis world linear acceleration values.
     * If the sum of the absolute values of the x and y axis exceed,
     * 0.01g, the motion state is indicated.
     * @return Returns true if the nav6 IMU is currently detecting motion.
     */
    public boolean isMoving()
    {
        return (getAverageFromWorldLinearAccelHistory() >= 0.01);
    }

    /**
     * Returns the current temperature (in degrees centigrade) reported by
     * the nav6 gyro/accelerometer circuit.
     * 
     * This value may be useful in order to perform advanced temperature-
     * dependent calibration.
     * @return The current temperature (in degrees centigrade).
     */
    public float getTempC()
    {
        return this.tempC;
    }
    
    //@Override
    protected void initIMU() {
        super.initIMU();
        worldLinearAccelHistory = new float[WORLD_LINEAR_ACCEL_HISTORY_LENGTH];
        initWorldLinearAccelHistory();
    }

    private void initWorldLinearAccelHistory(){
        Arrays.fill(worldLinearAccelHistory,0);
        nextWorldLinearAccelHistoryIndex = 0;
        worldLinearAccelerationRecentAvg = (float) 0.0;
    }
    
    private void updateWorldLinearAccelHistory( float x, float y, float z ){
        if (nextWorldLinearAccelHistoryIndex >= WORLD_LINEAR_ACCEL_HISTORY_LENGTH) {
            nextWorldLinearAccelHistoryIndex = 0;
        }
        worldLinearAccelHistory[nextWorldLinearAccelHistoryIndex] = Math.abs(x) + Math.abs(y);
        nextWorldLinearAccelHistoryIndex++;
    }
    
    public float getAverageFromWorldLinearAccelHistory(){
        float worldLinearAccelHistorySum = (float) 0.0;
        for (int i = 0; i < WORLD_LINEAR_ACCEL_HISTORY_LENGTH; i++) {
            worldLinearAccelHistorySum += worldLinearAccelHistory[i];
        }
        return worldLinearAccelHistorySum / WORLD_LINEAR_ACCEL_HISTORY_LENGTH;
    }

    private void setQuaternion(IMUProtocol.QuaternionUpdate raw_update) {
        synchronized (this) { // synchronized block
            
            float[] q = new float[4];
            float[] gravity = new float[3];
            //float[] euler = new float[3];
            float[] ypr = new float[3];
            float yawDegrees;
            float pitchDegrees;
            float rollDegrees;
            float linearAccelerationX;
            float linearAccelerationY;
            float linearAccelerationZ;
            float q2[] = new float[4];
            float qProduct[] = new float[4];
            float worldLinearAccelerationX;
            float worldLinearAccelerationY;
            float worldLinearAccelerationZ;
                       
            q[0] = ((float)raw_update.q1) / 16384.0f;
            q[1] = ((float)raw_update.q2) / 16384.0f;
            q[2] = ((float)raw_update.q3) / 16384.0f;
            q[3] = ((float)raw_update.q4) / 16384.0f;
            for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i]; // Range-check quaterions
            
            // below calculations are necessary for calculation of yaw/pitch/roll, 
            // and tilt-compensated compass heading
            
            // calculate gravity vector
            gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
            gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
            gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
            // calculate Euler angles
            // This code is here for reference, and is commented out for performance reasons
           
            //euler[0] = (float) MathUtils.atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
            //euler[1] = (float) -MathUtils.asin(2*q[1]*q[3] + 2*q[0]*q[2]);
            //euler[2] = (float) MathUtils.atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
  
            // calculate yaw/pitch/roll angles
            ypr[0] = (float) Math.atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
            ypr[1] = (float) Math.atan(gravity[0] / Math.sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
            ypr[2] = (float) Math.atan(gravity[1] / Math.sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
             
            yawDegrees = (float) (ypr[0] * (180.0/Math.PI)); 
            pitchDegrees = (float) (ypr[1] * (180.0/Math.PI)); 
            rollDegrees = (float) (ypr[2] * (180.0/Math.PI)); 
             
            // Subtract nav6 offset, and handle potential 360 degree wrap-around
            yawDegrees -= nav6_yaw_offset_degrees;
            if ( yawDegrees < -180 ) yawDegrees += 360;
            if ( yawDegrees > 180 ) yawDegrees -= 360;
             
            // calculate linear acceleration by 
            // removing the gravity component (+1g = +4096 in standard DMP FIFO packet)
             
            linearAccelerationX = (float) ((((float)raw_update.accel_x) / (32768.0 / accel_fsr_g)) - gravity[0]);
            linearAccelerationY = (float) ((((float)raw_update.accel_y) / (32768.0 / accel_fsr_g)) - gravity[1]);
            linearAccelerationZ = (float) ((((float)raw_update.accel_z) / (32768.0 / accel_fsr_g)) - gravity[2]); 
            
            // Calculate world-frame acceleration
            
            q2[0] = 0;
            q2[1] = linearAccelerationX;
            q2[2] = linearAccelerationY;
            q2[3] = linearAccelerationZ;
            
            // Rotate linear acceleration so that it's relative to the world reference frame
            
            // http://www.cprogramming.com/tutorial/3d/quaternions.html
            // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
            // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
            // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
        
            // P_out = q * P_in * conj(q)
            // - P_out is the output vector
            // - q is the orientation quaternion
            // - P_in is the input vector (a*aReal)
            // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])

            
            // calculate quaternion product
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            
            qProduct[0] = q[0]*q2[0] - q[1]*q2[1] - q[2]*q2[2] - q[3]*q2[3];  // new w
            qProduct[1] = q[0]*q2[1] + q[1]*q2[0] + q[2]*q2[3] - q[3]*q2[2];  // new x
            qProduct[2] = q[0]*q2[2] - q[1]*q2[3] + q[2]*q2[0] + q[3]*q2[1];  // new y 
            qProduct[3] = q[0]*q2[3] + q[1]*q2[2] - q[2]*q2[1] + q[3]*q2[0];  // new z

            float[] qConjugate = new float[4];
            
            qConjugate[0] = q[0];            
            qConjugate[1] = -q[1];            
            qConjugate[2] = -q[2];            
            qConjugate[3] = -q[3];            

            float[] qFinal = new float[4];
            
            qFinal[0] = qProduct[0]*qConjugate[0] - qProduct[1]*qConjugate[1] - qProduct[2]*qConjugate[2] - qProduct[3]*qConjugate[3];  // new w
            qFinal[1] = qProduct[0]*qConjugate[1] + qProduct[1]*qConjugate[0] + qProduct[2]*qConjugate[3] - qProduct[3]*qConjugate[2];  // new x
            qFinal[2] = qProduct[0]*qConjugate[2] - qProduct[1]*qConjugate[3] + qProduct[2]*qConjugate[0] + qProduct[3]*qConjugate[1];  // new y 
            qFinal[3] = qProduct[0]*qConjugate[3] + qProduct[1]*qConjugate[2] - qProduct[2]*qConjugate[1] + qProduct[3]*qConjugate[0];  // new z

            worldLinearAccelerationX = qFinal[1];
            worldLinearAccelerationY = qFinal[2];
            worldLinearAccelerationZ = qFinal[3];
             
            updateWorldLinearAccelHistory(worldLinearAccelerationX, worldLinearAccelerationY, worldLinearAccelerationZ);
             
            // Calculate tilt-compensated compass heading
            
            float invertedPitch = -ypr[1];
            float rollRadians = ypr[2];
            
            float cosRoll = (float) Math.cos(rollRadians);
            float sinRoll = (float) Math.sin(rollRadians);
            float cosPitch = (float) Math.cos(invertedPitch);
            float sinPitch = (float) Math.sin(invertedPitch);
            
            float MAG_X = raw_update.mag_x * cosPitch + raw_update.mag_z * sinPitch;
            float MAG_Y = raw_update.mag_x * sinRoll * sinPitch + raw_update.mag_y * cosRoll - raw_update.mag_z * sinRoll * cosPitch;
            float tiltCompensatedHeadingRadians = (float) Math.atan2(MAG_Y, MAG_X);
            float tiltCompensatedHeadingDegrees = (float) (tiltCompensatedHeadingRadians * (180.0 / Math.PI));
            
            // Adjust compass for board orientation,
            // and modify range from -180-180 to
            // 0-360 degrees
          
            tiltCompensatedHeadingDegrees -= 90.0;
            if (tiltCompensatedHeadingDegrees < 0) {
              tiltCompensatedHeadingDegrees += 360; 
            }
            
            this.yaw = yawDegrees;
            this.pitch = pitchDegrees;
            this.roll = rollDegrees;
            this.compass_heading = tiltCompensatedHeadingDegrees;
            
            this.worldLinearAccelX = worldLinearAccelerationX;
            this.worldLinearAccelY = worldLinearAccelerationY;
            this.worldLinearAccelZ = worldLinearAccelerationZ;
            this.tempC = raw_update.temp_c;
            updateYawHistory(this.yaw);            
        }
    }
}
