package org.team3467.robot2018.robot.autonomous;

import org.team3467.robot2018.subsystems.ArmLift.ArmLift;
import org.team3467.robot2018.subsystems.ArmLift.ArmLiftTransition;
import org.team3467.robot2018.subsystems.DriveBase.DriveStraight;
import org.team3467.robot2018.subsystems.DriveBase.DriveTurn;
import org.team3467.robot2018.subsystems.DriveBase.SetBrakeMode;
import org.team3467.robot2018.subsystems.Pneumatics.CloseHands;
import org.team3467.robot2018.subsystems.Pneumatics.OpenHands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *  Drive straight ahead for specified distance
 */

public class AutoRightSwitch extends CommandGroup {

    public  AutoRightSwitch() {

    	// Brakes off
    	addSequential(new SetBrakeMode(false));

    	// Drive to switch
    	addSequential(new DriveStraight(77000, 0.5));
    	
    	// Wait to settle
    	addSequential(new WaitCommand(1.0));    	
    	
    	// After we stop, turn brakes back on
   // 	addSequential(new SetBrakeMode(true));
    	
    	// Turn 90 degrees toward switch
    	addSequential(new DriveTurn(90.0, 0.35));
    	
    	// Raise arm
    	addSequential(new ArmLiftTransition(ArmLift.eArmLiftState.SwitchFront, false));

    	// Drive to switch
    	addSequential(new DriveStraight(10000, 0.4));
    	
    	// Wait to settle
    	addSequential(new WaitCommand(1.0));    	
    	
    	// Open hands, wait, then close
    	addSequential(new OpenHands());
    	addSequential(new WaitCommand(2.0));
    	addSequential(new CloseHands());
    	
    	// Back away
    	
    	// Turn toward scale
    	
    	
    }
}
