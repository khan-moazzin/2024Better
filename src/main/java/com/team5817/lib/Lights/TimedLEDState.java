package com.team5817.lib.Lights;

public enum TimedLEDState {
	OFF("OFF", Color.off()),
	DISABLE_BLUE("DISABLE_BLUE", 1.0, Color.GREEN_DIMMED, Color.BLUE_DIMMED),
	DISABLE_RED("DISABLE_RED", 1.0, Color.GREEN_DIMMED, Color.RED_DIMMED),
	NO_VISION("NO_VISION", Color.ORANGE),
	EMERGENCY("EMERGENCY", 0.2, Color.RED, Color.off()),
	INTAKING("INTAKING", Color.BLUE),
	PREPARED("PREPARED", Color.GREEN),
	IDLE("IDLE", Color.CYAN),
	HOLDING("HOLDING", Color.PURPLE),
	INDEXING("INDEXING",Color.GREEN_DIMMED),
	PREPARING("PREPARING", Color.ORANGE),
	WAITING_FOR_AUTO_ALIGN("WAITING_FOR_AUTO_ALIGN", Color.YELLOW);

	public final Color[] colors; // array of colors to iterate over
	public final double interval; // time in seconds between states
	public final String name; // name of state

	TimedLEDState(String name, double interval, Color... colors) {
		this.colors = colors;
		this.interval = interval;
		this.name = name;
	}

	TimedLEDState(String name, Color... colors) {
		this.colors = colors;
		this.interval = Double.POSITIVE_INFINITY;
		this.name = name;
	}

	public Color[] getColors() {
		return colors;
	}

	public double getInterval() {
		return interval;
	}

	public String getName() {
		return name;
	}
}
