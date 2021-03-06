package org.team3467.robot2018.subsystems.FieldCamera;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class FieldCamera extends Subsystem {

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public FieldCamera() {
    
    	// Run one USB camera
    	runOne();

    	// Run two USB cameras
    	// runTwo();
    }
    
    private void runOne() {
    	
		UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1.setResolution(320, 240);
        camera1.setFPS(30);
    
    }
    
    private void runTwo() {
       	
     	Thread t = new Thread(() -> {
			
			boolean allowCam1 = true;
			
			UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
	        camera1.setResolution(320, 240);
	        camera1.setFPS(20);
	        UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
	        camera2.setResolution(320, 240);
	        camera2.setFPS(20);
	        
	        CvSink cvSink = CameraServer.getInstance().getVideo(camera1);
	        CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 320, 240);
	        
	        Mat image = new Mat();
	        
	        while(!Thread.interrupted()) {
	        	
// TODO: Need OI method to get camera switch
//	        	if (CommandBase.oi.getGamepad().getRawButton(9)) {
//	        		allowCam1 = !allowCam1;
//	        	}
	        	
	            if (allowCam1){
	              cvSink.setSource(camera1);
	            } else{
	              cvSink.setSource(camera2);
	            }
	            
            	cvSink.grabFrame(image);
	            outputStream.putFrame(image);
	        }
	        
	    });

		t.start();
    }
    
}

