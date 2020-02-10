/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import java.text.BreakIterator;

import edu.wpi.first.wpilibj.command.Command;

public class XboxMove extends Command {
  //Input Axis

  double throttle;
  double turn;
  double reverse;


  //Buttons
  boolean rotate;
  boolean brake;
  boolean precision;
  boolean gearShiftLow;
  boolean gearShiftHigh;


  //Vars
  double sensitivity;
  double left;
  double right;

  


  @Override
  protected boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }


}
