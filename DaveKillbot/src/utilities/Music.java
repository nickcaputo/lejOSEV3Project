package utilities;

import lejos.hardware.Sound;

public class Music {
	
	public static void playDirectionTone(int orientation) {
		switch (orientation) {
		case 0:
			Sound.playTone(520, 150);
			break;
		case 1:
			Sound.playTone(622, 150);
			break;
		case 2:
			Sound.playTone(784, 150);
			break;
		case 3:
			Sound.playTone(932, 150);
		}
	}
	
	/**
	 * Plays the Legend of Zelda treasure chest item theme
	 */
	public static void playVictoryTuneOne() { // LoZ treasure chest item theme
		Sound.playTone(880, 252);
		Sound.playTone(932, 252);
		Sound.playTone(988, 252);
		Sound.playTone(1046, 750);
	}

	/**
	 * Plays the Legend of Zelda treasure chest item theme
	 */
	public static void playVictoryTuneTwo() {
		// TODO CHANGE THIS TUNE TO SOMETHING ELSE
		Sound.playTone(880, 252);
		Sound.playTone(932, 252);
		Sound.playTone(988, 252);
		Sound.playTone(1046, 750);
	}

}
