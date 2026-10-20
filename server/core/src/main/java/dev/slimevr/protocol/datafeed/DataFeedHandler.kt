package dev.slimevr.protocol.datafeed

import com.google.flatbuffers.FlatBufferBuilder
import dev.slimevr.protocol.DataFeed
import dev.slimevr.protocol.GenericConnection
import dev.slimevr.protocol.ProtocolAPI
import dev.slimevr.protocol.ProtocolAPIServer
import dev.slimevr.protocol.ProtocolHandler
import dev.slimevr.tracking.trackers.Tracker
import io.eiren.util.logging.LogManager
import solarxr_protocol.MessageBundle
import solarxr_protocol.data_feed.DataFeedConfigT
import solarxr_protocol.data_feed.DataFeedMessage
import solarxr_protocol.data_feed.DataFeedMessageHeader
import solarxr_protocol.data_feed.DataFeedUpdate
import solarxr_protocol.data_feed.PollDataFeed
import solarxr_protocol.data_feed.StartDataFeed
import java.util.function.Consumer
import java.util.stream.Collectors

class DataFeedHandler(private val api: ProtocolAPI) : ProtocolHandler<DataFeedMessageHeader>() {
	init {
		registerPacketListener(DataFeedMessage.StartDataFeed, ::onStartDataFeed)
		registerPacketListener(DataFeedMessage.PollDataFeed, ::onPollDataFeedRequest)
		this.api.server.addOnTick { this.sendDataFeedUpdate() }
	}

	private fun onStartDataFeed(conn: GenericConnection, header: DataFeedMessageHeader) {
		val req = header.message(StartDataFeed()) as StartDataFeed? ?: return
		val dataFeeds = req.dataFeedsLength()

		val feedList = conn.context.dataFeedList
		synchronized(feedList) {
			feedList.clear()
			for (i in 0..<dataFeeds) {
				// Using the object api here because we
				// need to copy from the buffer, anyway let's
				// do it from here and send the reference to an arraylist
				val config = req.dataFeeds(i).unpack()
				feedList.add(DataFeed(config))
			}
		}
	}

	private fun onPollDataFeedRequest(
		conn: GenericConnection,
		messageHeader: DataFeedMessageHeader,
	) {
		val req = messageHeader.message(PollDataFeed()) as PollDataFeed? ?: return

		val fbb = FlatBufferBuilder(300)

		val messageOffset = this.buildDatafeed(fbb, req.config().unpack(), 0)

		DataFeedMessageHeader.startDataFeedMessageHeader(fbb)
		DataFeedMessageHeader.addMessage(fbb, messageOffset)
		DataFeedMessageHeader.addMessageType(fbb, DataFeedMessage.DataFeedUpdate)
		val headerOffset = DataFeedMessageHeader.endDataFeedMessageHeader(fbb)

		MessageBundle.startDataFeedMsgsVector(fbb, 1)
		MessageBundle.addDataFeedMsgs(fbb, headerOffset)
		val datafeedMessagesOffset = fbb.endVector()

		val packet = createMessage(fbb, datafeedMessagesOffset)
		fbb.finish(packet)
		conn.send(fbb.dataBuffer())
	}

	fun buildDatafeed(fbb: FlatBufferBuilder, config: DataFeedConfigT, index: Int): Int {
		val devicesOffset = createDevicesData(
			fbb,
			config.dataMask,
			this.api.server.deviceManager
				.devices,
		)
		// Synthetic tracker is computed tracker apparently
		val trackersOffset = createSyntheticTrackersData(
			fbb,
			config.syntheticTrackersMask,
			this.api.server
				.allTrackers
				.stream()
				.filter(Tracker::isComputed)
				.collect(Collectors.toList()),
		)

		val h = this.api.server.humanPoseManager
		val bonesOffset =
			createBonesData(
				fbb,
				config.boneMask,
				h.allBones.toMutableList(),
			)

		var stayAlignedPoseOffset = 0
		if (config.stayAlignedPoseMask) {
			stayAlignedPoseOffset = createStayAlignedPose(fbb, this.api.server.humanPoseManager.skeleton)
		}

		var serverGuardsOffset = 0
		if (config.serverGuardsMask) {
			serverGuardsOffset = createServerGuard(fbb, this.api.server.serverGuards)
		}

		return DataFeedUpdate
			.createDataFeedUpdate(
				fbb,
				devicesOffset,
				trackersOffset,
				bonesOffset,
				stayAlignedPoseOffset,
				index,
				serverGuardsOffset,
			)
	}

	fun sendDataFeedUpdate() {
		val currTime = System.currentTimeMillis()

		this.api.apiServers.forEach(
			Consumer { server: ProtocolAPIServer ->
				server.apiConnections.forEach { conn: GenericConnection ->
					val feedList = conn.context.dataFeedList
					synchronized(feedList) {
						val configsCount = feedList.size
						val headers = ArrayList<Int>(configsCount)
						var fbb: FlatBufferBuilder? = null
						for (index in 0..<configsCount) {
							val feed = feedList[index]
							val lastTimeSent = feed.timeLastSent
							val configT = feed.config
							if (currTime - lastTimeSent > configT.minimumTimeSinceLast) {
								// Create the buffer only when there is something to send.
								val builder = fbb ?: FlatBufferBuilder(300).also { fbb = it }

								val messageOffset = this.buildDatafeed(builder, configT, index)

								DataFeedMessageHeader.startDataFeedMessageHeader(builder)
								DataFeedMessageHeader.addMessage(builder, messageOffset)
								DataFeedMessageHeader.addMessageType(builder, DataFeedMessage.DataFeedUpdate)
								headers.add(DataFeedMessageHeader.endDataFeedMessageHeader(builder))

								feed.timeLastSent = currTime
							}
						}
						val builder = fbb ?: return@synchronized

						// Only include headers that were actually built. A vector with
						// null entries is invalid Flat Buffers: strict verifiers reject
						// the entire frame, including its tracker data.
						val messages = MessageBundle.createDataFeedMsgsVector(builder, headers.toIntArray())
						val packet = createMessage(builder, messages)
						builder.finish(packet)
						conn.send(builder.dataBuffer())
					}
				}
			},
		)
	}

	override fun onMessage(conn: GenericConnection, message: DataFeedMessageHeader) {
		val consumer = this.handlers[message.messageType().toInt()]
		if (consumer != null) {
			consumer.accept(conn, message)
		} else {
			LogManager
				.info(
					"[ProtocolAPI] Unhandled Datafeed packet received id: " + message.messageType(),
				)
		}
	}

	override fun messagesCount(): Int = DataFeedMessage.names.size

	fun createMessage(fbb: FlatBufferBuilder, datafeedMessagesOffset: Int): Int {
		MessageBundle.startMessageBundle(fbb)
		if (datafeedMessagesOffset > -1) MessageBundle.addDataFeedMsgs(fbb, datafeedMessagesOffset)
		return MessageBundle.endMessageBundle(fbb)
	}
}
