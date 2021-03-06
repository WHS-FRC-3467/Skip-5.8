package org.team3467.robot2018.robot.control.triggers;

import org.team3467.robot2018.robot.control.Gamepad;

import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 *
 */
public class GamepadRightTrigger extends Trigger {
    
	Gamepad m_gamepad;
	public GamepadRightTrigger(Gamepad gamepad) {
		this.m_gamepad = gamepad;
	}	

    public boolean get() {
    		boolean rightTrigger = false;
    		if(m_gamepad.getRawAxis(Gamepad.rightTrigger_Axis) > .8 ){
    			rightTrigger = true;
    		}
    		else{
    			rightTrigger = false;
    		}
    		return rightTrigger;

    }
}
