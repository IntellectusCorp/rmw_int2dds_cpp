// Copyright 2026 Int2DDS Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Wait set tests that need no DDS participant.
///
/// rmw_create_wait_set and rmw_create_guard_condition only validate the context
/// implementation identifier, so guard condition behaviour can be exercised
/// without discovery, sockets or traffic. That keeps these cases deterministic
/// and fast, and they are what pins the readiness contract of rmw_wait: a guard
/// trigger must be reported exactly once, must not be consumed before the
/// blocking wait, and must never be lost.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "rmw/rmw.h"
#include "rmw/error_handling.h"

#include "rmw_int2dds_cpp/foxy_compat.hpp"  // RMW_DURATION_INFINITE on Foxy
#include "rmw_int2dds_cpp/identifier.hpp"

namespace
{

constexpr std::chrono::milliseconds kWatchdogTimeout{5000};

rmw_time_t
make_timeout(int64_t sec, uint64_t nsec)
{
  rmw_time_t t;
  t.sec = static_cast<uint64_t>(sec);
  t.nsec = nsec;
  return t;
}

/// Minimal context accepted by the wait set and guard condition factories.
rmw_context_t
make_stub_context()
{
  rmw_context_t context = rmw_get_zero_initialized_context();
  context.implementation_identifier = rmw_int2dds_cpp::implementation_identifier;
  return context;
}

/// Owns a wait set plus a fixed number of guard conditions and rebuilds the
/// rmw_guard_conditions_t handle array before every wait, the way rcl does.
class GuardFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = make_stub_context();
    wait_set_ = rmw_create_wait_set(&context_, 0);
    ASSERT_NE(nullptr, wait_set_);
  }

  void TearDown() override
  {
    for (auto * gc : guards_) {
      if (gc != nullptr) {
        EXPECT_EQ(RMW_RET_OK, rmw_destroy_guard_condition(gc));
      }
    }
    guards_.clear();
    if (wait_set_ != nullptr) {
      EXPECT_EQ(RMW_RET_OK, rmw_destroy_wait_set(wait_set_));
      wait_set_ = nullptr;
    }
  }

  rmw_guard_condition_t * add_guard()
  {
    rmw_guard_condition_t * gc = rmw_create_guard_condition(&context_);
    EXPECT_NE(nullptr, gc);
    guards_.push_back(gc);
    return gc;
  }

  /// rmw_wait receives implementation data pointers, not the rmw_* wrappers,
  /// and nulls the slots it considers not ready. Rebuild the array each call.
  void rebuild_handles()
  {
    handles_.clear();
    handles_.reserve(guards_.size());
    for (auto * gc : guards_) {
      handles_.push_back(gc->data);
    }
    gc_array_.guard_conditions = handles_.data();
    gc_array_.guard_condition_count = handles_.size();
  }

  rmw_ret_t wait(const rmw_time_t * timeout)
  {
    rebuild_handles();
    return rmw_wait(nullptr, &gc_array_, nullptr, nullptr, nullptr, wait_set_, timeout);
  }

  bool reported(size_t index) const
  {
    return handles_[index] != nullptr;
  }

  /// Block in rmw_wait until another thread triggers the first guard.
  ///
  /// A second guard acts as a watchdog escape: if the wait does not wake, the
  /// watchdog triggers it so the call returns and the test fails on an
  /// assertion instead of hanging the whole binary. rmw_trigger_guard_condition
  /// is warn_unused_result and a gtest assertion raised on a spawned thread
  /// cannot abort the test, so both return codes are recorded and asserted on
  /// the main thread after the joins.
  void expect_wakes_on_cross_thread_trigger(const rmw_time_t * timeout)
  {
    rmw_guard_condition_t * target = add_guard();
    rmw_guard_condition_t * rescue = add_guard();

    std::atomic<bool> watchdog_fired{false};
    std::atomic<bool> wait_returned{false};
    std::atomic<rmw_ret_t> trigger_ret{RMW_RET_OK};
    std::atomic<rmw_ret_t> rescue_ret{RMW_RET_OK};

    std::thread trigger_thread(
      [target, &trigger_ret]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        trigger_ret.store(rmw_trigger_guard_condition(target));
      });

    std::thread watchdog(
      [rescue, &watchdog_fired, &wait_returned, &rescue_ret]() {
        const auto deadline = std::chrono::steady_clock::now() + kWatchdogTimeout;
        while (!wait_returned.load() && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!wait_returned.load()) {
          watchdog_fired.store(true);
          rescue_ret.store(rmw_trigger_guard_condition(rescue));
        }
      });

    const rmw_ret_t ret = wait(timeout);
    wait_returned.store(true);

    trigger_thread.join();
    watchdog.join();

    EXPECT_EQ(RMW_RET_OK, trigger_ret.load());
    EXPECT_EQ(RMW_RET_OK, rescue_ret.load());
    EXPECT_FALSE(watchdog_fired.load()) <<
      "rmw_wait did not wake on a guard triggered from another thread";
    EXPECT_EQ(RMW_RET_OK, ret);
    EXPECT_TRUE(reported(0));
  }

  rmw_context_t context_;
  rmw_wait_set_t * wait_set_{nullptr};
  std::vector<rmw_guard_condition_t *> guards_;
  std::vector<void *> handles_;
  rmw_guard_conditions_t gc_array_{};
};

}  // namespace

/// A trigger must be reported once and consumed once. Reporting it twice means
/// the post-wait scan stopped consuming; reporting it zero times means the
/// pre-wait scan consumed it before the wait could observe it.
TEST_F(GuardFixture, GuardTriggeredReportedAndConsumedExactlyOnce)
{
  add_guard();
  const rmw_time_t poll = make_timeout(0, 0);

  ASSERT_EQ(RMW_RET_OK, rmw_trigger_guard_condition(guards_[0]));

  EXPECT_EQ(RMW_RET_OK, wait(&poll));
  EXPECT_TRUE(reported(0)) << "trigger was set but not reported";

  // Second wait sees a consumed trigger.
  EXPECT_EQ(RMW_RET_TIMEOUT, wait(&poll));
  EXPECT_FALSE(reported(0)) << "trigger was reported twice; it was never consumed";
}

TEST_F(GuardFixture, GuardNotTriggeredNotSpuriouslyReported)
{
  add_guard();
  const rmw_time_t poll = make_timeout(0, 0);

  EXPECT_EQ(RMW_RET_TIMEOUT, wait(&poll));
  EXPECT_FALSE(reported(0));
}

/// A trigger set before the call must be seen without burning the timeout.
/// The return code alone would pass even if the trigger were only noticed at
/// the deadline, so the elapsed time is the real assertion here.
TEST_F(GuardFixture, GuardTriggeredBeforeWaitReturnsPromptly)
{
  add_guard();
  const rmw_time_t two_seconds = make_timeout(2, 0);

  ASSERT_EQ(RMW_RET_OK, rmw_trigger_guard_condition(guards_[0]));

  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(RMW_RET_OK, wait(&two_seconds));
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_TRUE(reported(0));
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 200) <<
    "an already set trigger was not observed until the wait timed out";
}

TEST_F(GuardFixture, MultipleGuardsOnlyTriggeredReported)
{
  add_guard();
  add_guard();
  add_guard();
  const rmw_time_t poll = make_timeout(0, 0);

  ASSERT_EQ(RMW_RET_OK, rmw_trigger_guard_condition(guards_[0]));

  EXPECT_EQ(RMW_RET_OK, wait(&poll));
  EXPECT_TRUE(reported(0));
  EXPECT_FALSE(reported(1));
  EXPECT_FALSE(reported(2));

  ASSERT_EQ(RMW_RET_OK, rmw_trigger_guard_condition(guards_[1]));
  ASSERT_EQ(RMW_RET_OK, rmw_trigger_guard_condition(guards_[2]));

  EXPECT_EQ(RMW_RET_OK, wait(&poll));
  EXPECT_FALSE(reported(0)) << "guard 0 was consumed by the previous wait";
  EXPECT_TRUE(reported(1));
  EXPECT_TRUE(reported(2));
}

/// Nothing ready and a finite timeout: the call must actually block, then time
/// out with every slot nulled.
TEST_F(GuardFixture, FiniteTimeoutBlocksThenTimesOut)
{
  add_guard();
  const rmw_time_t three_hundred_ms = make_timeout(0, 300u * 1000u * 1000u);

  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(RMW_RET_TIMEOUT, wait(&three_hundred_ms));
  const auto elapsed_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
    .count();

  EXPECT_FALSE(reported(0));
  EXPECT_GE(elapsed_ms, 250) << "returned before the deadline";
  // Generous upper bound: loaded CI plus scheduler granularity.
  EXPECT_LT(elapsed_ms, 3000) << "overshot the deadline badly";
}

/// The lost wake-up regression test. A trigger raised from another thread while
/// the call is blocked on an infinite timeout must wake it.
///
/// A watchdog guard is attached alongside the target so a lost wake-up fails
/// the assertion instead of hanging the whole binary.
TEST_F(GuardFixture, GuardTriggeredFromThreadInfiniteTimeoutWakesUp)
{
  expect_wakes_on_cross_thread_trigger(nullptr);  // nullptr timeout == infinite
}

/// RMW_DURATION_INFINITE must be treated as infinite, not as a huge finite
/// timeout that overflows the nanosecond conversion.
TEST_F(GuardFixture, RmwDurationInfiniteTreatedAsInfinite)
{
  const rmw_time_t infinite = RMW_DURATION_INFINITE;
  expect_wakes_on_cross_thread_trigger(&infinite);
}

/// A trigger landing in the window between the readiness scan and the attach
/// must not be missed. A finite timeout keeps a miss to a timing failure rather
/// than a hang.
TEST_F(GuardFixture, TriggerDuringScanAttachWindowNotMissed)
{
  add_guard();
  const rmw_time_t one_second = make_timeout(1, 0);
  constexpr int kIterations = 200;

  for (int i = 0; i < kIterations; ++i) {
    rmw_guard_condition_t * gc = guards_[0];
    const int delay_us = (i * 7) % 200;
    std::atomic<rmw_ret_t> trigger_ret{RMW_RET_OK};

    std::thread trigger_thread(
      [gc, delay_us, &trigger_ret]() {
        std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
        trigger_ret.store(rmw_trigger_guard_condition(gc));
      });

    const auto t0 = std::chrono::steady_clock::now();
    const rmw_ret_t ret = wait(&one_second);
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
      .count();
    trigger_thread.join();

    ASSERT_EQ(RMW_RET_OK, trigger_ret.load()) << "iteration " << i << " failed to trigger";
    ASSERT_EQ(RMW_RET_OK, ret) << "iteration " << i << " missed the trigger";
    ASSERT_TRUE(reported(0)) << "iteration " << i << " did not report the guard";
    ASSERT_LT(elapsed_ms, 500) << "iteration " << i << " only recovered at the deadline";
  }
}

TEST_F(GuardFixture, ZeroEntitiesTimesOut)
{
  const rmw_time_t poll = make_timeout(0, 0);
  EXPECT_EQ(RMW_RET_TIMEOUT, wait(&poll));
}

TEST_F(GuardFixture, NullWaitSetIsRejected)
{
  const rmw_time_t poll = make_timeout(0, 0);
  EXPECT_EQ(
    RMW_RET_INVALID_ARGUMENT,
    rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &poll));
  rmw_reset_error();
}

TEST_F(GuardFixture, ForeignWaitSetIsRejected)
{
  const rmw_time_t poll = make_timeout(0, 0);
  const char * saved = wait_set_->implementation_identifier;
  wait_set_->implementation_identifier = "rmw_not_int2dds";

  EXPECT_EQ(
    RMW_RET_INCORRECT_RMW_IMPLEMENTATION,
    rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, wait_set_, &poll));
  rmw_reset_error();

  wait_set_->implementation_identifier = saved;
}
