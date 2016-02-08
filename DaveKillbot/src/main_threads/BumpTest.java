package main_threads;


import java.util.Random;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.hardware.sensor.EV3TouchSensor;

public class BumpTest {

	static DifferentialPilot robot;
	static RegulatedMotor leftMotor = Motor.C;
	static RegulatedMotor rightMotor = Motor.B;
	static EV3TouchSensor bumpSensor;
	
	public static void main(String[] args){
		robot = new DifferentialPilot(5.5, 14.5, leftMotor, rightMotor, false);
		robot.setTravelSpeed(30);
		robot.setRotateSpeed(60);
		
		bumpSensor = new EV3TouchSensor(SensorPort.S2);
		
		run();
	}
	
	public static void run(){
		SampleProvider bumpSampleProvider = bumpSensor.getTouchMode();
		float[] bumpSensorData = new float[bumpSampleProvider.sampleSize()];
		bumpSampleProvider.fetchSample(bumpSensorData, 0);
		
		Random randGen = new Random();

		robot.forward(); // moves robot forward until robot.stop() is called
		
		while(!Button.ENTER.isDown()){
			if(bumpSensorData[0] == 1){
				robot.stop();
				robot.travel(-10); // back up
				int rotationDegrees = (randGen.nextInt(180)+90);
				robot.rotate(rotationDegrees);
				robot.forward();
			}
			bumpSampleProvider.fetchSample(bumpSensorData, 0);
		}
	}

}
