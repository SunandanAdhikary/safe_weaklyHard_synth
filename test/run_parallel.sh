#!/bin/bash

# Number of cores to use (leave one core free for system)
N=$(($(nproc) - 1))

# Find all model files
mapfile -t models < <(find models -name "*.model")

# Run models in parallel
for ((i=0; i<${#models[@]}; i++)); do
    echo "Starting ${models[i]} on core $((i % N))"
    taskset -c $((i % N)) ../../flowstar-2.1.0/flowstar "${models[i]}" &
done

# Wait for all processes to complete
wait
echo "All models completed"