#pragma once
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

struct PairedBinaryBootstrap
{
    int samples = 0;
    double rate_pct = 0.0;
    double rate_low_pct = 0.0;
    double rate_high_pct = 0.0;
    double delta_pct = 0.0;
    double delta_low_pct = 0.0;
    double delta_high_pct = 0.0;
};

inline double metricPercentile(std::vector<double> values, double quantile)
{
    if (values.empty())
        return 0.0;

    std::sort(values.begin(), values.end());
    const size_t index = static_cast<size_t>(
        std::floor(
            std::clamp(quantile, 0.0, 1.0) *
            static_cast<double>(values.size() - 1)));
    return values[index];
}

inline PairedBinaryBootstrap pairedBinaryBootstrap(
    const std::vector<char> &outcomes,
    const std::vector<char> &reference,
    int resamples,
    unsigned seed)
{
    PairedBinaryBootstrap summary;
    if (outcomes.empty() || outcomes.size() != reference.size())
        return summary;
    summary.samples = static_cast<int>(outcomes.size());

    double outcome_sum = 0.0;
    double reference_sum = 0.0;
    for (size_t i = 0; i < outcomes.size(); ++i)
    {
        outcome_sum += outcomes[i] ? 1.0 : 0.0;
        reference_sum += reference[i] ? 1.0 : 0.0;
    }
    summary.rate_pct = 100.0 * outcome_sum / outcomes.size();
    summary.delta_pct =
        100.0 * (outcome_sum - reference_sum) / outcomes.size();

    std::mt19937 rng(seed);
    std::uniform_int_distribution<size_t> sample_index(
        0, outcomes.size() - 1);
    std::vector<double> rates;
    std::vector<double> deltas;
    rates.reserve(std::max(0, resamples));
    deltas.reserve(std::max(0, resamples));
    for (int sample = 0; sample < resamples; ++sample)
    {
        double sampled_outcomes = 0.0;
        double sampled_reference = 0.0;
        for (size_t draw = 0; draw < outcomes.size(); ++draw)
        {
            const size_t index = sample_index(rng);
            sampled_outcomes += outcomes[index] ? 1.0 : 0.0;
            sampled_reference += reference[index] ? 1.0 : 0.0;
        }
        rates.push_back(100.0 * sampled_outcomes / outcomes.size());
        deltas.push_back(
            100.0 * (sampled_outcomes - sampled_reference) /
            outcomes.size());
    }

    summary.rate_low_pct = metricPercentile(rates, 0.025);
    summary.rate_high_pct = metricPercentile(rates, 0.975);
    summary.delta_low_pct = metricPercentile(deltas, 0.025);
    summary.delta_high_pct = metricPercentile(deltas, 0.975);
    return summary;
}
