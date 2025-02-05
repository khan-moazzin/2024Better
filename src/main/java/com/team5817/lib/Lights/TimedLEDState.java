package com.team5817.lib.Lights;

/**
 * Enum representing different LED states with associated colors and intervals.
 */
public enum TimedLEDState {
	OFF("OFF", Color.off()),
	DISABLE_BLUE("DISABLE_BLUE", 1.0, Color.GREEN_DIMMED, Color.BLUE_DIMMED),
	DISABLE_RED("DISABLE_RED", 1.0, Color.GREEN_DIMMED, Color.RED_DIMMED),
	NO_VISION("NO_VISION", Color.ORANGE),
	EMERGENCY("EMERGENCY", 0.2, Color.RED, Color.off()),
	IDLE("IDLE", Color.CYAN),
	INTAKING("INTAKING", Color.BLUE),
	HOLDING("HOLDING", Color.PURPLE),
	INDEXING("INDEXING", Color.GREEN_DIMMED),
	PREPARED("PREPARED", Color.GREEN),
	PREPARING("PREPARING", Color.ORANGE),
	WAITING_FOR_AUTO_ALIGN("WAITING_FOR_AUTO_ALIGN", Color.YELLOW);

	/**
	 * Array of colors to iterate over.
	 */
	public final Color[] colors;

	/**
	 * Time in seconds between states.
	 */
	public final double interval;

	/**
	 * Name of the state.
	 */
	public final String name;

	/**
	 * Constructor for states with a specified interval.
	 *
	 * @param name the name of the state
	 * @param interval the time interval in seconds
	 * @param colors the colors associated with the state
	 */
	TimedLEDState(String name, double interval, Color... colors) {
		this.colors = colors;
		this.interval = interval;
		this.name = name;
	}

	/**
	 * Constructor for states with an infinite interval.
	 *
	 * @param name the name of the state
	 * @param colors the colors associated with the state
	 */
	TimedLEDState(String name, Color... colors) {
		this.colors = colors;
		this.interval = Double.POSITIVE_INFINITY;
		this.name = name;
	}

	/**
	 * Gets the colors associated with the state.
	 *
	 * @return the colors
	 */
	public Color[] getColors() {
		return colors;
	}

	/**
	 * Gets the interval time in seconds.
	 *
	 * @return the interval
	 */
	public double getInterval() {
		return interval;
	}

	/**
	 * Gets the name of the state.
	 *
	 * @return the name
	 */
	public String getName() {
		return name;
	}
}
