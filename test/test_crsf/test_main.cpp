// Host unit tests for lib/CrsfProtocol (run with `pio test -e native`).
#include <unity.h>

#include <cstddef>
#include <cstdint>

#include <crsf/Builder.h>
#include <crsf/Channels.h>
#include <crsf/Crc8.h>
#include <crsf/LinkStats.h>
#include <crsf/Parser.h>

namespace {

const crsf::Channels kChannels{{172, 992, 1811, 1500, 500, 1000, 992, 992, 172, 1811, 700, 1300, 992, 992, 992, 992}};

int feedAll(crsf::Parser& parser, const uint8_t* data, size_t size) {
    int frames = 0;
    for (size_t i = 0; i < size; ++i) {
        if (parser.feed(data[i])) {
            ++frames;
        }
    }
    return frames;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_crc8_check_value() {
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX8(0xBC, crsf::crc8(text, sizeof(text)));
    TEST_ASSERT_EQUAL_HEX8(0xBC, crsf::crc8("123456789"));
    TEST_ASSERT_EQUAL_HEX8(0x00, crsf::crc8(text, 0));
}

void test_ticks_to_us_vectors() {
    TEST_ASSERT_EQUAL_UINT16(988, crsf::ticksToUs(172));
    TEST_ASSERT_EQUAL_UINT16(1500, crsf::ticksToUs(992));
    TEST_ASSERT_EQUAL_UINT16(2011, crsf::ticksToUs(1811));
}

void test_us_to_ticks_round_trip() {
    for (uint16_t us = 988; us <= 2011; ++us) {
        const uint16_t back = crsf::ticksToUs(crsf::usToTicks(us));
        TEST_ASSERT_UINT16_WITHIN(1, us, back);
    }
}

void test_pack_unpack_round_trip() {
    const crsf::Channels out = crsf::unpackChannels(crsf::packChannels(kChannels));
    for (size_t i = 0; i < crsf::kChannelCount; ++i) {
        TEST_ASSERT_EQUAL_UINT16(kChannels[i], out[i]);
    }
}

void test_pack_matches_known_layout() {
    // Channel 0 = 0x7FF fills bits 0..10: byte0 = 0xFF, byte1 low 3 bits set.
    crsf::Channels channels{};
    channels[0]                  = 0x7FF;
    const crsf::ChannelsPayload p = crsf::packChannels(channels);
    TEST_ASSERT_EQUAL_HEX8(0xFF, p[0]);
    TEST_ASSERT_EQUAL_HEX8(0x07, p[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, p[2]);
}

void test_valid_frame_parsed_once_on_last_byte() {
    const crsf::RcChannelsFrame frame = crsf::buildRcChannelsFrame(kChannels);
    crsf::Parser                parser;
    for (size_t i = 0; i + 1 < frame.size(); ++i) {
        TEST_ASSERT_FALSE(parser.feed(frame[i]));
    }
    TEST_ASSERT_TRUE(parser.feed(frame[frame.size() - 1]));
    TEST_ASSERT_EQUAL_UINT8(0x16, parser.type());
    TEST_ASSERT_EQUAL_size_t(crsf::kChannelsPayload, parser.payloadSize());
    TEST_ASSERT_EQUAL_UINT32(0, parser.errors());

    crsf::Channels ticks{};
    crsf::unpackChannels(parser.payload(), ticks);
    for (size_t i = 0; i < crsf::kChannelCount; ++i) {
        TEST_ASSERT_EQUAL_UINT16(kChannels[i], ticks[i]);
    }
}

void test_bad_crc_rejected() {
    crsf::RcChannelsFrame frame = crsf::buildRcChannelsFrame(kChannels);
    frame[10] ^= 0x04;
    crsf::Parser parser;
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, frame.data(), frame.size()));
    TEST_ASSERT_EQUAL_UINT32(1, parser.errors());
}

void test_bad_length_rejected() {
    crsf::Parser  parser;
    const uint8_t tooShort[] = {crsf::kSyncByte, 0x01};
    const uint8_t tooLong[]  = {crsf::kSyncByte, 0x3F};
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, tooShort, sizeof(tooShort)));
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, tooLong, sizeof(tooLong)));
    TEST_ASSERT_EQUAL_UINT32(2, parser.errors());

    // The parser is clean afterwards.
    const crsf::RcChannelsFrame frame = crsf::buildRcChannelsFrame(kChannels);
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame.data(), frame.size()));
}

void test_resync_after_garbage() {
    const uint8_t garbage[] = {0x00, 0xFF, crsf::kSyncByte, 0x00, 0x13, crsf::kSyncByte, 0x7F, 0x42, 0xC8};
    crsf::Parser  parser;
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, garbage, sizeof(garbage)));

    // The trailing 0xC8 above was taken as a sync byte; feeding a frame from its length byte on completes it.
    const crsf::RcChannelsFrame frame = crsf::buildRcChannelsFrame(kChannels);
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame.data() + 1, frame.size() - 1));
    TEST_ASSERT_EQUAL_size_t(crsf::kChannelsPayload, parser.payloadSize());
}

void test_split_frame() {
    const crsf::RcChannelsFrame frame = crsf::buildRcChannelsFrame(kChannels);
    crsf::Parser                parser;
    TEST_ASSERT_EQUAL_INT(0, feedAll(parser, frame.data(), 10));
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame.data() + 10, frame.size() - 10));
}

void test_back_to_back_frames() {
    const crsf::RcChannelsFrame frame = crsf::buildRcChannelsFrame(kChannels);
    uint8_t                     stream[2 * crsf::kRcChannelsFrameSize];
    for (size_t i = 0; i < frame.size(); ++i) {
        stream[i]                = frame[i];
        stream[frame.size() + i] = frame[i];
    }
    crsf::Parser parser;
    TEST_ASSERT_EQUAL_INT(2, feedAll(parser, stream, sizeof(stream)));
    TEST_ASSERT_EQUAL_UINT32(0, parser.errors());
}

void test_sync_byte_in_length_slot_restarts_frame() {
    const crsf::RcChannelsFrame frame = crsf::buildRcChannelsFrame(kChannels);
    crsf::Parser                parser;
    TEST_ASSERT_FALSE(parser.feed(crsf::kSyncByte));  // stray sync byte right before a real frame
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame.data(), frame.size()));
}

void test_link_stats_decode() {
    const uint8_t   payload[crsf::kLinkStatsPayload] = {70, 80, 99, 0xF6, 1, 4, 3, 60, 100, 0x05};
    const crsf::LinkStats stats                       = crsf::decodeLinkStats(payload);
    TEST_ASSERT_EQUAL_UINT8(70, stats.uplinkRssi1);
    TEST_ASSERT_EQUAL_UINT8(80, stats.uplinkRssi2);
    TEST_ASSERT_EQUAL_UINT8(99, stats.uplinkLq);
    TEST_ASSERT_EQUAL_INT8(-10, stats.uplinkSnr);
    TEST_ASSERT_EQUAL_UINT8(1, stats.activeAntenna);
    TEST_ASSERT_EQUAL_UINT8(4, stats.rfMode);
    TEST_ASSERT_EQUAL_UINT8(3, stats.uplinkTxPower);
    TEST_ASSERT_EQUAL_UINT8(60, stats.downlinkRssi);
    TEST_ASSERT_EQUAL_UINT8(100, stats.downlinkLq);
    TEST_ASSERT_EQUAL_INT8(5, stats.downlinkSnr);
}

void test_link_stats_frame_through_parser() {
    uint8_t frame[crsf::kHeaderSize + 1 + crsf::kLinkStatsPayload + 1] = {
        crsf::kSyncByte, 12, 0x14, 70, 80, 99, 0xF6, 1, 4, 3, 60, 100, 0x05, 0};
    frame[sizeof(frame) - 1] = crsf::crc8(&frame[2], 1 + crsf::kLinkStatsPayload);
    crsf::Parser parser;
    TEST_ASSERT_EQUAL_INT(1, feedAll(parser, frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_UINT8(0x14, parser.type());
    TEST_ASSERT_EQUAL_size_t(crsf::kLinkStatsPayload, parser.payloadSize());
    TEST_ASSERT_EQUAL_UINT8(99, crsf::decodeLinkStats(parser.payload()).uplinkLq);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_check_value);
    RUN_TEST(test_ticks_to_us_vectors);
    RUN_TEST(test_us_to_ticks_round_trip);
    RUN_TEST(test_pack_unpack_round_trip);
    RUN_TEST(test_pack_matches_known_layout);
    RUN_TEST(test_valid_frame_parsed_once_on_last_byte);
    RUN_TEST(test_bad_crc_rejected);
    RUN_TEST(test_bad_length_rejected);
    RUN_TEST(test_resync_after_garbage);
    RUN_TEST(test_split_frame);
    RUN_TEST(test_back_to_back_frames);
    RUN_TEST(test_sync_byte_in_length_slot_restarts_frame);
    RUN_TEST(test_link_stats_decode);
    RUN_TEST(test_link_stats_frame_through_parser);
    return UNITY_END();
}
