package main_threads;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class WalkForwardUntilDist {

	private static int sWidth;
	private static int sHeight;
	private static GraphicsLCD screen;

	private static DifferentialPilot roboKiller;
	private static RegulatedMotor leftMotor = Motor.C;
	private static RegulatedMotor rightMotor = Motor.B;
	private static EV3TouchSensor bumper;

	private static EV3UltrasonicSensor distSense;
	private static float[] samples;

	public static void main(String[] args) {

		setUpScreen();

		setUpMotors();

		setUpBumper();

		setUpDistanceSensor();

		run();
	}

	/**
	 * Runs the program in this class
	 * 
	 * Right now it makes the robot move forward until you press the Enter
	 * button on it
	 */
	private static void run() {
		SampleProvider bumpSample = bumper.getTouchMode();
		float[] bumperData = new float[bumpSample.sampleSize()];
		bumpSample.fetchSample(bumperData, 0);

		while (!Button.ENTER.isDown()) {
			roboKiller.forward();
		}

	}

	/**
	 * Sets up the LCD display for use in this class
	 */
	private static void setUpScreen() {
		screen = BrickFinder.getDefault().getGraphicsLCD();
		sWidth = screen.getWidth();
		sHeight = screen.getHeight();
		screen.setFont(Font.getSmallFont());

		write("Initialized Screen");
	}

	/**
	 * Sets up the motors for use
	 */
	private static void setUpMotors() {
		roboKiller = new DifferentialPilot(5.5, 14.5, leftMotor, rightMotor);
		roboKiller.setTravelSpeed(60.0);
		roboKiller.setRotateSpeed(61.1);

		write("Killbot motors initialized.");
	}

	/**
	 * Sets up the bumper for use
	 */
	private static void setUpBumper() {
		bumper = new EV3TouchSensor(SensorPort.S2);

		write("Bumper initialized.");
	}

	/**
	 * Sets up the Ultrasonic sensor for use
	 */
	private static void setUpDistanceSensor() {
		distSense = new EV3UltrasonicSensor(SensorPort.S1);
		distSense.enable();

		SampleProvider sampleProvider = distSense.getDistanceMode();

		samples = new float[sampleProvider.sampleSize()];
	}

	/**
	 * Writes a message to the LCD screen in the center of the display
	 * 
	 * @param message
	 */
	private static void write(String message) {
		screen.drawString(message, sWidth / 2, sHeight / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	}

}
