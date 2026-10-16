// ================================================================
// inertial_runner_probe.cpp
// 설명: IMU CSV 검증과 영상 시각별 배치 경계를 실제 C++ helper로 검사한다.
// 사용법: rover_imu_input_probe [imu.csv pairs.tsv]
// ================================================================
#include "inertial_input.h"
#include "map_export.h"
#include <deque>
#include <functional>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

const char* header = "timestamp_ns,gx_rad_s,gy_rad_s,gz_rad_s,ax_m_s2,ay_m_s2,az_m_s2\n";

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void rejected(const std::function<void()>& action, const std::string& reason) {
    try { action(); } catch (const std::runtime_error& error) {
        require(std::string(error.what()).find(reason) != std::string::npos, error.what());
        return;
    }
    throw std::runtime_error("invalid input accepted: " + reason);
}

std::vector<rover::ImuSample> parse(const std::string& rows) {
    std::istringstream input(std::string(header) + rows);
    return rover::read_imu_csv(input);
}

std::vector<std::int64_t> pinned_preintegrator(std::deque<std::int64_t>& queue,
        std::int64_t previous, std::int64_t current) {
    // pinned Tracking::PreintegrateIMU의 pop/retain 순서를 재현한다.
    std::vector<std::int64_t> integrated;
    while (!queue.empty()) {
        const auto stamp = queue.front();
        if (stamp < previous - 1000000) queue.pop_front();
        else if (stamp < current - 1000000) {
            integrated.push_back(stamp);
            queue.pop_front();
        } else {
            integrated.push_back(stamp);
            break;
        }
    }
    return integrated;
}

std::size_t check_pinned_queue(const std::vector<rover::ImuSample>& samples,
        const std::vector<std::int64_t>& frames) {
    rover::ImuBatches batches(samples);
    std::deque<std::int64_t> queue;
    std::int64_t previous = -1, last_submitted = -1;
    std::size_t processed = 0;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto batch = batches.next_for_orb(frames[i]);
        if (batch.empty()) continue;
        for (const auto& sample : batch) {
            require(sample.stamp_ns > last_submitted, "IMU sample submitted more than once or out of order");
            queue.push_back(sample.stamp_ns);
            last_submitted = sample.stamp_ns;
        }
        if (previous >= 0) {
            const auto integrated = pinned_preintegrator(queue, previous, frames[i]);
            require(integrated.size() >= 2, "native queue lost minimum integration interval");
            for (std::size_t j = 1; j < integrated.size(); ++j)
                require(integrated[j] > integrated[j - 1], "native queue interpolation has zero/negative dt");
        }
        previous = frames[i]; // 실제 core도 첫 영상은 queue를 유지한다.
        ++processed;
    }
    return processed;
}

int main(int argc, char** argv) {
    try {
        if (argc == 3) {
            std::ifstream imu(argv[1]), pairs(argv[2]);
            require(imu.good() && pairs.good(), "cannot open replay fixture");
            const auto samples = rover::read_imu_csv(imu);
            std::vector<std::int64_t> frames;
            std::string line;
            while (std::getline(pairs, line)) frames.push_back(std::stoll(line.substr(0, line.find('\t'))));
            const auto processed = check_pinned_queue(samples, frames);
            std::cout << "Pinned queue replay passed: " << processed << "/" << frames.size() << " frames\n";
            return 0;
        }
        int reset_map = 231, other_map = 300;
        std::map<unsigned long, int*> observed_maps;
        for (unsigned long i = 0; i < 9; ++i) observed_maps[i * 23] = &reset_map;
        const auto reset_exports = rover::unique_map_pointers(observed_maps);
        require(reset_exports.size() == 1 && reset_exports.front() == &reset_map,
            "reset map exported repeatedly through stale map IDs");
        observed_maps[300] = &other_map;
        require(rover::unique_map_pointers(observed_maps).size() == 2,
            "distinct final map omitted from export");
        const auto samples = parse("900000000,1,2,3,4,5,6\n950000000,0,0,0,0,0,9.81\n1000000000,0,0,0,0,0,9.81\n1050000000,0,0,0,0,0,9.81\n1100000000,0,0,0,0,0,9.81\n1150000000,0,0,0,0,0,9.81\n");
        require(samples.size() == 6, "CSV dropped samples");
        require(samples[0].stamp_ns == 900000000 && samples[0].gyro[2] == 3 && samples[0].accel[2] == 6, "CSV units/order changed");
        check_pinned_queue(samples, {1025000000, 1075000000, 1125000000});
        rover::ImuBatches submitted(samples);
        const auto first_submitted = submitted.next_for_orb(1000000000);
        require(first_submitted.size() == 3 && first_submitted.back().stamp_ns == 1050000000,
            "first ORB submission needs the actual right bracket sample");
        const auto unique = submitted.next_for_orb(1100000000);
        require(unique.size() == 2 && unique[0].stamp_ns == 1100000000 && unique[1].stamp_ns == 1150000000,
            "ORB internally retained boundary must not be submitted twice");
        check_pinned_queue(samples, {1000000000, 1050000000, 1100000000});
        check_pinned_queue(samples, {1000500000, 1050500000, 1100500000});
        check_pinned_queue(samples, {1025000000, 1125000000});
        rover::ImuBatches short_interval(samples);
        short_interval.next_for_orb(1000000000);
        require(short_interval.next_for_orb(1001000000).empty(), "unsafe effective integration interval must be skipped");
        const auto resumed = short_interval.next_for_orb(1050000000);
        require(resumed.size() == 1 && resumed.front().stamp_ns == 1100000000,
            "skipping must preserve unique future-sample submission cursor");
        rejected([&] { submitted.next_for_orb(1100000000); }, "frame timestamps");
        rejected([&] { submitted.next_for_orb(1300000000); }, "coverage");
        rover::ImuBatches asynchronous(samples);
        const auto first_between = asynchronous.next_for_orb(1025000000);
        require(first_between.size() == 3 && first_between[0].stamp_ns == 950000000
            && first_between[2].stamp_ns == 1050000000, "asynchronous first frame needs two preceding samples and right bracket");
        rover::ImuBatches sparse(samples);
        rejected([&] { sparse.next_for_orb(900000000); }, "preceding");
        rejected([&] { sparse.next_for_orb(925000000); }, "preceding");
        rejected([] { parse("100,0,0,0,0,0,1\n100,0,0,0,0,0,1\n"); }, "strictly increasing");
        rejected([] { parse("100,0,0,0,0,0,1\n99,0,0,0,0,0,1\n"); }, "strictly increasing");
        rejected([] { parse("100,0,0,0,0,0,1\n100000101,0,0,0,0,0,1\n"); }, "100 ms");
        require(parse("100,0,0,0,0,0,1\n100000100,0,0,0,0,0,1\n").size() == 2, "100 ms boundary rejected");
        rejected([] { parse("100,nan,0,0,0,0,1\n"); }, "finite");
        rejected([] { parse("100,0,inf,0,0,0,1\n"); }, "finite");
        rejected([] { parse("100,0,0,0,0,0,1,2\n"); }, "seven columns");
        rejected([] { parse("100,0,0,0,0,1\n"); }, "seven columns");
        rejected([] { parse("100x,0,0,0,0,0,1\n"); }, "timestamp");
        rejected([] { parse("100,0x,0,0,0,0,1\n"); }, "numeric");
        rejected([] { parse("-100,0,0,0,0,0,1\n"); }, "timestamp");
        rejected([] { parse(""); }, "empty");
        rejected([] { std::istringstream input("stamp,gx,gy,gz,ax,ay,az\n"); rover::read_imu_csv(input); }, "header");
        std::cout << "IMU CSV and frame boundary checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
