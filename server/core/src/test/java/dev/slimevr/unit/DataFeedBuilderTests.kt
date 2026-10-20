package dev.slimevr.unit

import com.google.flatbuffers.FlatBufferBuilder
import dev.slimevr.protocol.datafeed.createDevicesData
import dev.slimevr.tracking.trackers.Device
import dev.slimevr.tracking.trackers.DeviceOrigin
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import solarxr_protocol.data_feed.DataFeedUpdate
import solarxr_protocol.data_feed.device_data.DeviceDataMaskT
import solarxr_protocol.data_feed.tracker.TrackerDataMaskT

class DataFeedBuilderTests {

	private fun mask() = DeviceDataMaskT().apply {
		deviceData = true
		trackerData = TrackerDataMaskT()
	}

	@Test
	fun `devices without trackers are not serialized as null offsets`() {
		val trackers = TestTrackerSet()
		val withTracker = Device(DeviceOrigin.UDP).apply {
			this.trackers[0] = trackers.hip
		}
		val withoutTrackers = Device(DeviceOrigin.UDP)

		val fbb = FlatBufferBuilder(300)
		val devicesOffset = createDevicesData(
			fbb,
			mask(),
			mutableListOf(withoutTrackers, withTracker),
		)
		val update = DataFeedUpdate.createDataFeedUpdate(fbb, devicesOffset, 0, 0, 0, 0, 0)
		fbb.finish(update)

		val parsed = DataFeedUpdate.getRootAsDataFeedUpdate(fbb.dataBuffer())
		assertEquals(1, parsed.devicesLength())
		assertEquals(withTracker.id, parsed.devices(0)?.id()?.id())
	}

	@Test
	fun `device order is preserved after filtering`() {
		val trackers = TestTrackerSet()
		val first = Device(DeviceOrigin.UDP).apply {
			this.trackers[0] = trackers.hip
		}
		val empty = Device(DeviceOrigin.UDP)
		val second = Device(DeviceOrigin.UDP).apply {
			this.trackers[0] = trackers.chest
		}

		val fbb = FlatBufferBuilder(300)
		val devicesOffset = createDevicesData(
			fbb,
			mask(),
			mutableListOf(first, empty, second),
		)
		val update = DataFeedUpdate.createDataFeedUpdate(fbb, devicesOffset, 0, 0, 0, 0, 0)
		fbb.finish(update)

		val parsed = DataFeedUpdate.getRootAsDataFeedUpdate(fbb.dataBuffer())
		assertEquals(2, parsed.devicesLength())
		assertEquals(first.id, parsed.devices(0)?.id()?.id())
		assertEquals(second.id, parsed.devices(1)?.id()?.id())
	}
}
