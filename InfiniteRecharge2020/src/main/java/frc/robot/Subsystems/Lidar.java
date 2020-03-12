package frc.robot.Subsystems;

import java.util.TimerTask;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.MeasureDistance;

public class Lidar extends Subsystem {
	private I2C i2c;
	private byte[] distanceArray;
	private boolean started;
	
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	public Lidar(Port port) {
		i2c = new I2C(port, LIDAR_ADDR);
		
		distanceArray = new byte[2];
		
    }
    
    @Override
	public void initDefaultCommand() {
		setDefaultCommand(new MeasureDistance());
		//Default command not needed, called from OI.
	}
	
	// Distance in cm
	public int convertDistance() {
		return (int)Integer.toUnsignedLong(distanceArray[0] << 8) + Byte.toUnsignedInt(distanceArray[1]);
	}

	public double getDistance() {
		return (((double)convertDistance())/100);
	}
    
    public void start()
    {
        started = true;
    }
	
	public void stop() {
		started = false;
	}
	
	// Update distance variable
	public void update() {
		System.out.println("Yes way jose");
        if (started == true) {
			i2c.write(0x04, 0x08 | 32);
			i2c.write(0x11, 0xff);
            i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // Initiate measurement
            Timer.delay(0.04); // Delay for measurement to be taken
            i2c.read(LIDAR_DISTANCE_REGISTER, 2, distanceArray); // Read in measurement
			Timer.delay(0.02); // Delay to prevent over polling
			System.out.println("no way jose");
        }    
    }
    public void reportLidarDistance()
    {
        SmartDashboard.putNumber("Lidar Distance", getDistance());
    }
	
}    

    

