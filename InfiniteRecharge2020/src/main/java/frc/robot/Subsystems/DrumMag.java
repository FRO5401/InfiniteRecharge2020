/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Commands.DrumControl;

/**
 * Add your docs here.
 */
public class DrumMag extends Subsystem {
  private DigitalInput lemon1, lemon2, lemon3, lemon4, lemon5;
  private DigitalInput kicker;
  private int magMode;
  public boolean target;

  boolean[] lemonLimits;

    public DrumMag(){
      lemon1 = new DigitalInput(0);
      lemon2 = new DigitalInput(0);
      lemon3 = new DigitalInput(0);
      lemon4 = new DigitalInput(0);
      lemon5 = new DigitalInput(0);

      magMode = 1; //Itz da shootuh

    }

    public void changeMode()
    {
      magMode = 1 - magMode;
    }

    public void rotate ()
    {

    }

    public int findDesiredPosition(){
      int desiredPosition;
      lemonLimits[0] = lemon1.get();
      lemonLimits[1] = lemon2.get();
      lemonLimits[2] = lemon3.get();
      lemonLimits[3] = lemon4.get();
      lemonLimits[4] = lemon5.get();

      target = true;
      if(magMode == 1){
        target = true;
      }
      else if(magMode == 0){
        target = false;
      }

      int firstValueIndex = IntStream.range(0, lemonLimits.length).filter(i -> target == lemonLimits[i]).findFirst().orElse(-1);
      desiredPosition = ((2 * firstValueIndex) + (1 - magMode) * 5) % 10;
      return desiredPosition;      
    }

    public boolean[] getLemonLimits(){
      return lemonLimits;
    }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DrumControl());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }



}
