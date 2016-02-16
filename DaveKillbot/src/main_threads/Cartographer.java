package main_threads;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
//import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import utilities.Music;
import utilities.Write;

/**
 * Roves about a maze from a designated start point to locate a goal, then
 * returns to the starting position, documenting a map along the way.
 * 
 * @author CGreen
 * @author npoCaputo
 *
 */
public class Cartographer {
	private static DifferentialPilot robotPilot;
	private static RegulatedMotor frontNeckMotor = Motor.A;
	private static EV3UltrasonicSensor distanceSensor;
	//private static EV3TouchSensor bumpSensor;
	private static EV3ColorSensor colorSensor;
	//private static SampleProvider bumpSampleProvider;
	private static SampleProvider distanceSampleProvider;
	private static GPS gps;
	private static GraphicsLCD mazeScreen;
	private static ArrayList<Integer[]> cellHistory;

	private static final int STARTING_X = 0;
	private static final int STARTING_Y = 0;
	private static final double MAX_DISTANCE = 1.23;
	private static final double TRAVEL_DISTANCE = 117;
	private static final int TRAVEL_SPEED = 30;
	private static final int ROTATE_SPEED = 60;
	private static final int ACCELERATION = 60;
	private static final int ROTATION_CORRECTION = 10;

	public static void main(String[] args) {

		main.start();

		stopper.start();

	}

	/**
	 * Rove the map in search of the goal, documenting the maze layout along the
	 * way.
	 * 
	 * @return the coordinates of the cell containing the goal.
	 */
	public static int[] findGoal() {
		gps.prepGPS();
		mazeScreen.drawString("Home: ", mazeScreen.getWidth() * 3 / 4 - 3, 10, 0);
		mazeScreen.drawString("(" + STARTING_X + "," + STARTING_Y + ")", mazeScreen.getWidth() * 3 / 4 - 3, 26, 0);
		int orientation = gps.getOrientation();

		int[] currentPosition;
		//bumpSampleProvider = bumpSensor.getTouchMode();
		//float[] bumpSample = new float[bumpSampleProvider.sampleSize()];
		float[] distanceScanData = new float[4];

		//bumpSampleProvider.fetchSample(bumpSample, 0);

		// looking for white
		while (colorSensor.getColorID() != Color.WHITE) {
			currentPosition = gps.getCoordinates();

			// Keep track that the robot came from the reverse direction
			gps.setDirectionTaken(currentPosition[0], currentPosition[1], (orientation + 2) % 4);

			// Gather data if cell is unfamiliar
			if (!gps.getVisited(currentPosition[0], currentPosition[1])) {
				scanCell(orientation, distanceScanData);
			}

			// Determine new direction of travel
			int newOrientation = getNewHeading(currentPosition, orientation);
			Music.playDirectionTone(newOrientation);

			// Rotate if necessary.
			rotateRobot(orientation, newOrientation);
			orientation = newOrientation;
			gps.updateOrientation(orientation);
			
			// If this is the first visit to this cell, make note that it has
			// been here.
			if (!gps.getVisited(currentPosition[0], currentPosition[1])) {
				gps.setVisited(currentPosition[0], currentPosition[1]);
			}

			// Add this cell to the history.
			cellHistory.add(new Integer[] { currentPosition[0], currentPosition[1] });

			// Make note of what direction was chosen.
			gps.setDirectionTaken(currentPosition[0], currentPosition[1], orientation);

			// Take note that the robot moved.
			gps.updatePosition(1);

			// Make yo' move.
			robotPilot.travel(TRAVEL_DISTANCE, true);

			while (robotPilot.isMoving()) {

				/*bumpSampleProvider.fetchSample(bumpSample, 0);

				if (bumpSample[0] == 1) { // is the touch sensor currently
											// pushed in?
					robotPilot.stop();
					robotPilot.travel(-10);
					robotPilot.rotate(-10); // always turn 90 degrees when you
											// bump into something? NO WAY MAN
				}*/

				if (colorSensor.getColorID() == Color.WHITE) { // found the GOAL
					break;

				}

			}
		}

		return foundGoal(orientation, distanceScanData);
	}

	/**
	 * Conducts functions that are set upon finding the goal, and returns the
	 * coordinates of the goal space.
	 * 
	 * @param orientation,
	 *            the orientation of where the robot is facing
	 * @param distanceScanData,
	 *            the distances in front of the robot
	 * @return the coordinates of the goal space
	 */
	private static int[] foundGoal(int orientation, float[] distanceScanData) {
		robotPilot.stop();
		Button.LEDPattern(4); // victory celebration!
		Music.playVictoryTuneOne();

		// TODO: Figure out how much make-up distance is required (set to 18 for now)
		robotPilot.travel(18);

		// Scan the cell containing the goal
		scanCell(orientation, distanceScanData);

		int[] goalCoords = gps.getCoordinates();
		gps.setVisited(goalCoords[0], goalCoords[1]);
		gps.setDirectionTaken(goalCoords[0], goalCoords[1], (orientation + 2) % 4);
		return goalCoords;
	}

	/**
	 * Scan the current cell to detect walls and update the GPS.
	 * 
	 * @param orientation - direction the robot's body is facing
	 * @param distanceScanData - data from the distance scanner
	 */
	public static void scanCell(int orientation, float[] distanceScanData) {
		int direction = orientation;
		int backwards = (orientation + 2) % 4;
		int[] coords = gps.getCoordinates();
		int count = 0;
		while (count < 4) {
			if (Button.ENTER.isDown()) {
				return;
			} // break out, if someone is holding down the ENTER button.

			distanceScanData[count] = getDistanceMeasurement();
			frontNeckMotor.rotate(90);
			count++;
		}
		BitSet wallInfo = new BitSet(4);
		for (int counter = 0; counter < 4; counter++) {
			if (distanceScanData[counter] < MAX_DISTANCE && direction != backwards) {
				wallInfo.set(direction);
			}
			direction = (direction + 1) % 4;
		}
		frontNeckMotor.rotate(-360); // rotate neck back (otherwise cords will
										// tighten/tangle).
		gps.setWalls(coords[0], coords[1], wallInfo);
	}

	/**
	 * Detects the distance of obstacles (if any) in a cardinal direction.
	 * 
	 * @return the distance between a perceived object, if any
	 */
	public static float getDistanceMeasurement() {
		float[] distanceSample = new float[distanceSampleProvider.sampleSize()];
		distanceSampleProvider.fetchSample(distanceSample, 0);
		// provide instant feedback on whether a wall was detected
		if (distanceSample[0] < MAX_DISTANCE) {
			Sound.playTone(66, 100);
		} else {
			Sound.playTone(523, 100);
		}
		return distanceSample[0];
	}
	
	/**
	 * Rotates the robot from one direction to another most efficiently.
	 * 
	 * @param oldOrientation - Direction robot currently faces
	 * @param newOrientation - Direction robot faces following rotation
	 */
	public static void rotateRobot(int oldOrientation, int newOrientation) {
		if (newOrientation != oldOrientation) {
			int rotationAngle = (oldOrientation - newOrientation) * 90;
			if (Math.abs(rotationAngle) == 270) {
				if (oldOrientation - newOrientation < 0) {
					rotationAngle += 360;
				} else {
					rotationAngle -= 360;
				}
			}
			// TODO: Figure out better correction methods.
			robotPilot.rotate(rotationAngle);
			if (rotationAngle < 0) {
				robotPilot.rotate(ROTATION_CORRECTION);
			} else {
				robotPilot.rotate(-ROTATION_CORRECTION);
			}
		}
	}

	/**
	 * Determine the next direction to travel based on data from scans.
	 * 
	 * @param coordinates - Coordinates of the current cell
	 * @param currentOrientation - current direction the robot is facing
	 * 
	 * @return the new direction that ought to be travelled
	 */
	public static int getNewHeading(int[] coordinates, int currentOrientation) {
		int wallCount = 3;
		int newOrientation = currentOrientation;
		int backwards = (currentOrientation + 2) % 4;
		BitSet wallInfo = gps.getWalls(coordinates[0], coordinates[1]);
		BitSet pathsTaken = gps.getDirectionsTaken(coordinates[0], coordinates[1]);

		/* Check directions starting with the direction TO THE LEFT of where the
		* robot's body is facing. For example, if the robot is facing south, the robot
		* will check the data in index 1 first. (0=N, 1=E, 2=S, 3=W)
		*/
		
		int direction = (currentOrientation + 3) % 4;
		for (int counter = 0; counter < 3; counter++) {
			if (!wallInfo.get(direction)) {
				wallCount--;

				// If the robot has been here before, we will try and travel
				// in a direction not previously taken, still (hopefully)
				// following DFS.
				
				if (gps.getVisited(coordinates[0], coordinates[1])) {
					if (!pathsTaken.get(direction)) {
						newOrientation = direction;
						return newOrientation;
					}
					// Otherwise, we check for a new cell to visit.
				} else {
					if (!gps.getVisitedNeighbor(direction)) {
						newOrientation = direction;
						return newOrientation;
					}
				}
			}
			direction = (direction + 1) % 4;
		}

		/* If no good options: if only choices are one way or backwards, goes the
		* one way. If two or more options, takes the leftmost path.
		* Turns around if all other options fail (i.e. dead end).
		*/
		
		direction = (currentOrientation + 3) % 4;

		if (wallCount == 2 || wallCount == 1) {
			for (int counter = 0; counter < 3; counter++) {
				if (!wallInfo.get(direction) && direction != backwards) {
					return direction;
				}
				direction = (direction + 1) % 4;
			}
		}

		return (currentOrientation + 2) % 4;
	}

	/**
	 * Go back to the starting cell.
	 */
	public static void goHome() {
		ArrayList<Integer[]> stepsHome = new ArrayList<Integer[]>();
		for (int index = 0; index < cellHistory.size(); index++) {
			if (isInArrayList(cellHistory.get(index), stepsHome)) {
				Integer[] duplicate = cellHistory.get(index);
				stepsHome.add(0, duplicate);
				int indexOfDupe = findFirstInstance(duplicate, stepsHome);
				if (indexOfDupe == -1) {
					throw new ArrayIndexOutOfBoundsException("First coordinate instance was not found.");
				}
				stepsHome.remove(indexOfDupe);
				while (!Arrays.equals(duplicate, stepsHome.get(indexOfDupe))) {
					stepsHome.remove(indexOfDupe);
				}
			} else {
				stepsHome.add(0, cellHistory.get(index));
			}
		}
		int[] homeCoords = {STARTING_X, STARTING_Y};
		while (gps.getCoordinates()!=homeCoords) {
			int[] neighbor = {stepsHome.get(0)[0], stepsHome.get(0)[1]};
			goToNeighbor(neighbor);
			if (stepsHome.size() > 1) {
				stepsHome.remove(0);
			}
		}
	}

	/**
	 * Sets up global variables for first use
	 */
	private static void setUpGlobals() {
		robotPilot = new DifferentialPilot(5.4, 14.5, Motor.C, Motor.B, false);
		distanceSensor = new EV3UltrasonicSensor(SensorPort.S1);
		//bumpSensor = new EV3TouchSensor(SensorPort.S2);
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		distanceSampleProvider = distanceSensor.getDistanceMode();

		mazeScreen = BrickFinder.getDefault().getGraphicsLCD();
		gps = new GPS(mazeScreen, STARTING_X, STARTING_Y, 0, 4, 4);
		cellHistory = new ArrayList<Integer[]>();
	}

	/**
	 * Configures settings for the DifferentialPilot
	 */
	private static void configureRobot() {
		robotPilot.setTravelSpeed(TRAVEL_SPEED);
		robotPilot.setAcceleration(ACCELERATION);
		robotPilot.setRotateSpeed(ROTATE_SPEED);
		robotPilot.reset();
	}
	
	/**
	 * When going back to the start position, tells the robot to move to the given cell.
	 * The cell should be directly adjacent to the current cell.
	 * 
	 * @param coordinates - Array containing the x and y coordinates of the destination.
	 */
	public static void goToNeighbor(int[] coordinates) {
		int[] currentPosition = gps.getCoordinates();
		
		// returns if current coords equal destination, or cells are not directly adjacent
		if ((coordinates[0] == currentPosition[0]) == (coordinates[1] == currentPosition[1])) {
			return;
		}
		int newOrientation;
		if (coordinates[0] == currentPosition[0]) { // if the x values are the same
			if (coordinates[1] < currentPosition[1]) {
				newOrientation = 2;
			} else {
				newOrientation = 0;
			}
		} else { // if the y values are the same
			if (coordinates[0] < currentPosition[0]) {
				newOrientation = 3;
			} else {
				newOrientation = 1;
			}
		}
		rotateRobot(gps.getOrientation(), newOrientation);
		robotPilot.travel(TRAVEL_DISTANCE);
		gps.updateOrientation(newOrientation);
		gps.updatePosition(1);
	}
	
	/**
	 * Determines whether a particular coordinate is present in the list of coordinates the
	 * robot must take to reach home.
	 * 
	 * This circumvents the faulty results from .equals() and .contains() methods for the
	 * Integer[] objects.
	 * 
	 * @param cellCoords - cell coordinates in question
	 * @param stepsHome - list of coordinates robot must visit to return home
	 * @return
	 */
	public static boolean isInArrayList(Integer[] cellCoords, ArrayList<Integer[]> stepsHome) {
		for (int index = 0; index < stepsHome.size(); index++) {
			if (Arrays.equals(cellCoords, stepsHome.get(index))) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Finds the index of the first instance of a coordinate pair in the steps
	 * to get home.
	 * 
	 * This circumvents the faulty results from the .indexOf() method for the
	 * Integer[] objects.
	 * 
	 * @param cellCoords - cell coordinates in question
	 * @param stepsHome - list of coordinates robot must visit to return home
	 * @return
	 */
	public static int findFirstInstance(Integer[] cellCoords, ArrayList<Integer[]> stepsHome) {
		for (int index = 0; index < stepsHome.size(); index++) {
			if (Arrays.equals(cellCoords, stepsHome.get(index))) {
				return index;
			}
		}
		return -1;
	}
	
	

	/*
	 * 
	 * 
	 * 
	 * 
	 * 
	 * ********************* THREADS ************************
	 * 
	 * 
	 * 
	 * 
	 * 
	 * 
	 */

	/**
	 * The main thread which starts all accessories, started from the main()
	 * method
	 */
	private static Thread main = new Thread() {
		@Override
		public void run() {
			// Sets up Pilot for robot and sensors for robot
			setUpGlobals();

			configureRobot();

			Write.writeLines("Press Enter to", "begin Dave's", "mission.");

			Button.waitForAnyPress();

			Write.showCountdown(mazeScreen, 5, (short) 1000);

			int[] goalLocation = findGoal();

			mazeScreen.drawString("Goal: ", mazeScreen.getWidth() * 3 / 4 - 3, 50, 0);
			mazeScreen.drawString("(" + goalLocation[0] + "," + goalLocation[1] + ")",
					mazeScreen.getWidth() * 3 / 4 - 3, 66, 0);

			Delay.msDelay(1000);
			
			

			goHome();
			
			Music.playVictoryTuneTwo();
			
			Button.waitForAnyPress();
			
			Write.clearScreen();
		}
	};

	/**
	 * The thread which allows us to stop at any point by pressing the ESCAPE
	 * button. Started from the final method in the main thread.
	 */
	private static Thread stopper = new Thread() {
		@Override
		public void run() {
			while (true) {

				Button.waitForAnyPress();

				if (Button.ESCAPE.isDown()) {
					System.exit(0);
				}
			}
		}
	};

}
