#!/bin/bash

# Create parameter combinations
create_params() {
    for K in 10 15 20; do
        for M in 5 8 10; do
            echo "-k $K -m $M"
        done
    done
}

# Export the parameters to a temporary file
create_params > params.txt

# Run with GNU Parallel
parallel -j$(nproc) '../../flowstar-2.1.0/flowstar models/sus_ctrl/sus_test.model {}' :::: params.txt

# Clean up
rm params.txt