package org.team3467.robot2018.robot.autonomous;

import org.team3467.robot2018.subsystems.ArmLift.ArmLift;
import org.team3467.robot2018.subsystems.ArmLift.ArmLiftTransition;
import org.team3467.robot2018.subsystems.DriveBase.DriveStraight;
import org.team3467.robot2018.subsystems.DriveBase.SetBrakeMode;
import org.team3467.robot2018.subsystems.Pneumatics.CloseHands;
import org.team3467.robot2018.subsystems.Pneumatics.OpenHands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *  Drive straight ahead and drop cube in Switch
 */

public class AutoLeftSwitch_Front extends CommandGroup {

    public  AutoLeftSwitch_Front() {

    	// Brakes off
    	addSequential(new SetBrakeMode(false));

    	// Drive to switch
    	addSequential(new DriveStraight(65000, 0.65));
    	
    	// Raise arm
    	addSequential(new ArmLiftTransition(ArmLift.eArmLiftState.SwitchFront, false));

    	// Wait to settle
    	addSequential(new WaitCommand(3.0));    	
    	
    	// Open hands, wait, then close
    	addSequential(new OpenHands());
    	addSequential(new WaitCommand(2.0));
    	addSequential(new CloseHands());
    	
    	
    }
}
