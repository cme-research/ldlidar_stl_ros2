// Protocol-layer tests for the LDROBOT LD06/LD19 driver (no serial port).
// Synthetic, correctly CRC'd data packets are fed byte-wise into LiPkg exactly
// as the serial reader does, and the assembled scan is checked.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include "lipkg.h"

namespace ldlidar
{
// Defined in lipkg.cpp (not declared in a header).
uint8_t CalCRC8(const uint8_t * data, uint16_t data_len);
}  // namespace ldlidar

using ldlidar::LDType;
using ldlidar::LiDARFrameTypeDef;
using ldlidar::LiPkg;
using ldlidar::Points2D;

namespace
{

constexpr uint16_t kSpeedDegPerSec = 3600;    // 10 Hz
constexpr int kPointsPerRev = 450;            // LD19: 4500 pts/s at 10 Hz
constexpr double kStepDeg = 360.0 / kPointsPerRev;

// Independent bitwise CRC-8 (poly 0x4D, init 0, no reflection) to validate the table.
uint8_t crc8_bitwise(const uint8_t * data, size_t len)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

/// Distance pattern used by the synthetic LiDAR: a function of the point index.
/// Periodic per revolution so every turn shows the same scene.
uint16_t distance_for(int point_index)
{
  return static_cast<uint16_t>(500 + (point_index % kPointsPerRev) * 5);
}

std::vector<uint8_t> make_packet(int first_point, bool corrupt = false)
{
  LiDARFrameTypeDef pkt{};
  pkt.header = ldlidar::PKG_HEADER;
  pkt.ver_len = ldlidar::PKG_VER_LEN;
  pkt.speed = kSpeedDegPerSec;
  const double start = std::fmod(first_point * kStepDeg, 360.0);
  const double end = std::fmod((first_point + ldlidar::POINT_PER_PACK - 1) * kStepDeg, 360.0);
  pkt.start_angle = static_cast<uint16_t>(std::lround(start * 100));
  pkt.end_angle = static_cast<uint16_t>(std::lround(end * 100));
  for (int i = 0; i < ldlidar::POINT_PER_PACK; ++i) {
    pkt.point[i].distance = distance_for(first_point + i);
    pkt.point[i].intensity = 200;
  }
  pkt.timestamp = static_cast<uint16_t>(first_point);
  auto * raw = reinterpret_cast<uint8_t *>(&pkt);
  pkt.crc8 = ldlidar::CalCRC8(raw, sizeof(pkt) - 1);
  std::vector<uint8_t> bytes(raw, raw + sizeof(pkt));
  if (corrupt) {
    bytes[10] ^= 0x01;  // flip one bit in a distance field
  }
  return bytes;
}

class LiPkgTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pkg_.SetProductType(LDType::LD_19);
    pkg_.EnableFilter(false);
    pkg_.RegisterTimestampGetFunctional([this]() {return clock_ns_ += 100000;});
  }

  /// Feed `revolutions` full turns of packets; optional garbage between packets.
  void feed(int revolutions, bool corrupt = false, bool garbage = false)
  {
    std::mt19937 rng(42);
    for (int p = 0; p < revolutions * kPointsPerRev; p += ldlidar::POINT_PER_PACK) {
      auto bytes = make_packet(p, corrupt);
      if (garbage) {
        std::vector<uint8_t> noise(7);
        // Random line noise without the packet header byte.
        for (auto & b : noise) {
          do {b = static_cast<uint8_t>(rng());} while (b == ldlidar::PKG_HEADER);
        }
        pkg_.CommReadCallback(reinterpret_cast<const char *>(noise.data()), noise.size());
      }
      // Deliver in two chunks, like a serial read that splits a packet.
      const size_t split = bytes.size() / 3;
      pkg_.CommReadCallback(reinterpret_cast<const char *>(bytes.data()), split);
      pkg_.CommReadCallback(
        reinterpret_cast<const char *>(bytes.data() + split), bytes.size() - split);
    }
  }

  LiPkg pkg_;
  uint64_t clock_ns_ = 0;
};

}  // namespace

TEST(Crc8, TableMatchesBitwiseImplementation)
{
  std::mt19937 rng(7);
  for (int n = 0; n < 200; ++n) {
    std::vector<uint8_t> data(1 + rng() % 64);
    for (auto & b : data) {b = static_cast<uint8_t>(rng());}
    ASSERT_EQ(ldlidar::CalCRC8(data.data(), data.size()), crc8_bitwise(data.data(), data.size()));
  }
}

TEST(Crc8, PacketLayoutIs47Bytes)
{
  EXPECT_EQ(sizeof(LiDARFrameTypeDef), 47u);  // LD06/LD19 data packet size on the wire
}

TEST_F(LiPkgTest, AssemblesAFullRevolution)
{
  feed(3);
  Points2D scan;
  ASSERT_TRUE(pkg_.GetLaserScanData(scan)) << "no complete frame after 3 revolutions";
  EXPECT_TRUE(pkg_.GetLidarPowerOnCommStatus());
  EXPECT_DOUBLE_EQ(pkg_.GetSpeed(), 10.0);

  // One revolution, give or take a packet at the seam.
  EXPECT_NEAR(static_cast<int>(scan.size()), kPointsPerRev, ldlidar::POINT_PER_PACK);
  for (const auto & pt : scan) {
    ASSERT_GE(pt.angle, 0.0f);
    ASSERT_LT(pt.angle, 360.0f);
    const int idx = static_cast<int>(std::lround(pt.angle / kStepDeg)) % kPointsPerRev;
    // Distances must belong to the angle they were measured at (within one step
    // of rounding in the packet's centi-degree angles).
    const bool matches = pt.distance == distance_for(idx) ||
      pt.distance == distance_for((idx + kPointsPerRev - 1) % kPointsPerRev) ||
      pt.distance == distance_for((idx + 1) % kPointsPerRev);
    ASSERT_TRUE(matches) << "angle " << pt.angle << " distance " << pt.distance;
    EXPECT_EQ(pt.intensity, 200);
  }
  for (size_t i = 1; i < scan.size(); ++i) {
    ASSERT_LE(scan[i - 1].stamp, scan[i].stamp) << "points not ordered by time";
  }
}

TEST_F(LiPkgTest, RejectsPacketsWithBadCrc)
{
  feed(3, /*corrupt=*/true);
  Points2D scan;
  EXPECT_FALSE(pkg_.GetLaserScanData(scan));
  EXPECT_FALSE(pkg_.GetLidarPowerOnCommStatus());
}

TEST_F(LiPkgTest, ResynchronisesAfterGarbageBytes)
{
  feed(3, /*corrupt=*/false, /*garbage=*/true);
  Points2D scan;
  ASSERT_TRUE(pkg_.GetLaserScanData(scan));
  EXPECT_NEAR(static_cast<int>(scan.size()), kPointsPerRev, ldlidar::POINT_PER_PACK);
}
