package dev.slimevr.config

/**
 * Live tracker data recording, used to analyse drift correction and tracking
 * accuracy on real hardware.
 *
 * The server hosts a small page on localhost that starts/stops a recording and
 * downloads the resulting JSON Lines files.
 */
class RecordingConfig {
	/** Host the recording page on localhost. */
	var enabled = true

	/** Port of the recording page. */
	var port = 21111

	/** Samples per second per tracker. */
	var sampleRateHz = 30

	/** Folder for recordings, null for the default (next to the logs). */
	var directory: String? = null

	/** Stop a recording once the file grows past this size. */
	var maxFileBytes = 256 * 1024 * 1024
}
