#include <gtest/gtest.h>

#include "scheduler/global_orchestrator.hpp"
#include "scheduler/time_budget.hpp"

using namespace gd2026::scheduler;

TEST(TimeAllocatorTest, EmptyBatchReturnsEmptyVector) {
    const std::vector<GraphMetadata> batch;
    const auto allocated = TimeAllocator::allocate_time(batch, 1000);
    EXPECT_TRUE(allocated.empty());
}

TEST(TimeAllocatorTest, AllocatesFullBudgetAndPreservesOrdering) {
    const std::vector<GraphMetadata> batch = {
        {0, 16, 8},
        {1, 64, 32},
        {2, 4, 4},
        {3, 100, 50}
    };

    const int64_t total_time = 5000;
    const auto allocated = TimeAllocator::allocate_time(batch, total_time);

    ASSERT_EQ(allocated.size(), batch.size());

    int64_t sum = 0;
    for (const int64_t value : allocated) {
        EXPECT_GE(value, 0);
        sum += value;
    }

    EXPECT_EQ(sum, total_time);
    EXPECT_GT(allocated[1], allocated[2]);
    EXPECT_GT(allocated[3], allocated[0]);
}

TEST(TimeAllocatorTest, CoarsePhaseTimeLeavesMajorityForFineGraph) {
    const GraphMetadata coarse{0, 500, 1200};
    const GraphMetadata fine{0, 10000, 20000};

    const int64_t coarse_budget = TimeAllocator::allocate_coarse_phase_time(coarse, fine, 1000);
    EXPECT_GE(coarse_budget, 150);
    EXPECT_LE(coarse_budget, 350);
    EXPECT_LT(coarse_budget, 500);
}

TEST(TimeAllocatorTest, CoarsePhaseTimeStillCapsEqualSizeGraphs) {
    const GraphMetadata same_a{0, 1000, 4000};
    const GraphMetadata same_b{0, 1000, 4000};

    const int64_t coarse_budget = TimeAllocator::allocate_coarse_phase_time(same_a, same_b, 1000);
    EXPECT_EQ(coarse_budget, 350);
}

TEST(GlobalOrchestratorTest, ThreadAllocationCapsDominantGraphInLargeBatch) {
    GlobalOrchestrator orchestrator(128);
    const std::vector<GraphMetadata> metadata = {
        {0, 50, 202},
        {1, 144, 314},
        {2, 100, 420},
        {3, 196, 571},
        {4, 500, 2556},
        {5, 200, 3000},
        {6, 500, 1740},
        {7, 10466, 20288},
        {8, 2519, 4938}
    };

    const std::vector<int32_t> allocations = orchestrator.allocate_threads(metadata);
    ASSERT_EQ(allocations.size(), metadata.size());

    int32_t total = 0;
    int32_t max_allocation = 0;
    for (const int32_t allocation : allocations) {
        total += allocation;
        max_allocation = std::max(max_allocation, allocation);
        EXPECT_GE(allocation, 1);
    }

    EXPECT_EQ(total, 128);
    EXPECT_LE(max_allocation, 32);
    EXPECT_GT(allocations[7], allocations[4]);
    EXPECT_GT(allocations[8], 1);
}

TEST(GlobalOrchestratorTest, SingleGraphKeepsAllAvailableThreads) {
    GlobalOrchestrator orchestrator(16);
    const std::vector<GraphMetadata> metadata = {
        {0, 1000, 2000}
    };

    const std::vector<int32_t> allocations = orchestrator.allocate_threads(metadata);
    ASSERT_EQ(allocations.size(), 1u);
    EXPECT_EQ(allocations[0], 16);
}

TEST(GlobalOrchestratorTest, ThreeGraphBatchStillUsesAllThreads) {
    GlobalOrchestrator orchestrator(128);
    const std::vector<GraphMetadata> metadata = {
        {0, 10466, 20288},
        {1, 2519, 4938},
        {2, 500, 2556}
    };

    const std::vector<int32_t> allocations = orchestrator.allocate_threads(metadata);
    ASSERT_EQ(allocations.size(), metadata.size());

    int32_t total = 0;
    for (const int32_t allocation : allocations) {
        total += allocation;
    }

    EXPECT_EQ(total, 128);
    EXPECT_LE(allocations[0], 43);
    EXPECT_LE(allocations[1], 43);
    EXPECT_LE(allocations[2], 43);
}
