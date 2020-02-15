package frc.robot.Subsystems;

import java.util.TimerTask;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lidar extends Subsystem {
	private I2C i2c;
	private byte[] distance;
    private boolean started;
	
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	public Lidar(Port port) {
		i2c = new I2C(port, LIDAR_ADDR);
		
		distance = new byte[2];
    }
    
    @Override
	public void initDefaultCommand() {
		//Default command not needed, called from OI.
	}
	
	// Distance in cm
	public int getDistance() {
		return (int)Integer.toUnsignedLong(distance[0] << 8) + Byte.toUnsignedInt(distance[1]);
	}

	public double pidGet() {
		return getDistance();
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
        if (started) {
            i2c.write(LIDAR_CONFIG_REGISTER, 0x04); // Initiate measurement
            Timer.delay(0.04); // Delay for measurement to be taken
            i2c.read(LIDAR_DISTANCE_REGISTER, 2, distance); // Read in measurement
            Timer.delay(0.005); // Delay to prevent over polling
        }    
    }
    public void reportLidarDistance()
    {
        SmartDashboard.putNumber("Lidar Distance", pidGet());
    }
	
}    

    

