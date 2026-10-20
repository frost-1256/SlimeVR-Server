package dev.slimevr.tracking.processor.stayaligned

import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.math.Angle
import dev.slimevr.tracking.processor.stayaligned.StayAlignedDefaults.IMU_TO_YAW_CORRECTION
import dev.slimevr.tracking.processor.stayaligned.StayAlignedDefaults.YAW_CORRECTION_DEFAULT
import dev.slimevr.tracking.processor.stayaligned.adjust.AdjustTrackerYaw
import dev.slimevr.tracking.processor.stayaligned.trackers.TrackerSkeleton

/**
 * Manager to keep the trackers aligned.
 */
object StayAligned {

	private var nextTrackerIndex = 0

	/**
	 * Adjusts the yaw of the next tracker.
	 *
	 * We only adjust one tracker per tick to minimize CPU usage. When the server is
	 * running at 1000 Hz and there are 20 trackers, each tracker is still updated 50
	 * times a second.
	 */
	fun adjustNextTracker(trackers: TrackerSkeleton, config: StayAlignedConfig) {
		if (!config.enabled) {
			return
		}

		val numTrackers = trackers.allTrackers.size
		if (numTrackers == 0) {
			return
		}

		val index = nextTrackerIndex % numTrackers
		val trackerToAdjust = trackers.allTrackers[index]
		nextTrackerIndex = (index + 1) % numTrackers

		val state = trackerToAdjust.stayAligned

		// Update hide correction since the config could have changed
		state.hideCorrection = config.hideYawCorrection

		// Use the measured interval for this tracker rather than the server frame
		// time times the tracker count. The latter drifts whenever the loop lags or
		// the tracker list changes, which directly skewed the correction rate.
		val dtSeconds = state.elapsedSinceAdjustment()

		val yawCorrectionPerSec =
			IMU_TO_YAW_CORRECTION.getOrDefault(trackerToAdjust.imuType, YAW_CORRECTION_DEFAULT)
		if (yawCorrectionPerSec == Angle.ZERO) {
			return
		}

		AdjustTrackerYaw.adjust(
			trackerToAdjust,
			trackers,
			yawCorrectionPerSec * dtSeconds,
			dtSeconds,
			config,
		)
	}
}
