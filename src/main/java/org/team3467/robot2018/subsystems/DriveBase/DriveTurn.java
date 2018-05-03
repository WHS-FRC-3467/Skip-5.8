package org.team3467.robot2018.subsystems.DriveBase;

import org.team3467.robot2018.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveTurn extends Command {

	private static final double TOLERANCE = 2;
	
	private PIDController m_pid;
	private double m_maxSpeed = 0.3;
	private double m_turnAngle = 0.0;
	
	private double KP = 1.0;
	private double KI = 0.0;
	private double KD = 2.0;
	
    public DriveTurn(double angle, double maxspeed) {
    	requires(Robot.driveBase);
		
		m_maxSpeed = maxspeed;
		m_turnAngle = angle;
		
		buildController();
    }
    
    private void buildController() {
		
		m_pid = new PIDController(KP, KI, KD,
                new PIDSource() {
                    PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

                    public double pidGet() {
                    	return Robot.imu.getCurrentAngle();
                    }

                    public void setPIDSourceType(PIDSourceType pidSource) {
                    	m_sourceType = pidSource;
                    }

                    public PIDSourceType getPIDSourceType() {
                        return m_sourceType;
                    }
                },
                new PIDOutput() {
                	
                	public void pidWrite(double d) {
                		// Drive with the magnitude returned by the PID calculation, 
                		// and spin around the center axis
            			Robot.driveBase.drive(0, -d, true);
                }});
		
        m_pid.setAbsoluteTolerance(TOLERANCE);
        m_pid.setOutputRange((m_maxSpeed * -1.0), m_maxSpeed);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    	m_pid.setSetpoint(Robot.imu.getCurrentAngle() + m_turnAngle);
      	m_pid.reset();
    	m_pid.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
 
    	Robot.driveBase.reportEncoders();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

    	double error = m_pid.getError();
   		return ((error >= 0 && error <= TOLERANCE) || (error < 0 && error >= (-1.0)*TOLERANCE));
    }

    // Called once after isFinished returns true
    protected void end() {
    	m_pid.disable();
        Robot.driveBase.drive(0, 0, false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
