package dev.slimevr.unit

import dev.slimevr.tracking.trackers.TrackerResetsHandler
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

/**
 * Tests the bounded drift compensation ratio.
 */
class DriftCompensationTests {

	@Test
	fun `correction saturates after one window without prediction`() {
		Assertions.assertEquals(0.5f, TrackerResetsHandler.driftTimeRatio(1_000, 2_000, false), 1e-4f)
		Assertions.assertEquals(1.0f, TrackerResetsHandler.driftTimeRatio(10_000, 2_000, false), 1e-4f)
	}

	@Test
	fun `prediction is bounded to one extra window`() {
		Assertions.assertEquals(1.5f, TrackerResetsHandler.driftTimeRatio(30_000, 20_000, true), 1e-4f)
		Assertions.assertEquals(2.0f, TrackerResetsHandler.driftTimeRatio(600_000, 20_000, true), 1e-4f)
	}

	@Test
	fun `non positive inputs produce no correction`() {
		Assertions.assertEquals(0.0f, TrackerResetsHandler.driftTimeRatio(0, 20_000, true))
		Assertions.assertEquals(0.0f, TrackerResetsHandler.driftTimeRatio(-100, 20_000, true))
		Assertions.assertEquals(0.0f, TrackerResetsHandler.driftTimeRatio(100, 0, true))
	}
}
