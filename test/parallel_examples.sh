#!/bin/bash

# Example 1: Run multiple model files in parallel
echo "Example 1: Running multiple model files"
find models -name "*.model" | parallel -j$(nproc) --bar '../../flowstar-2.1.0/flowstar {}'

# Example 2: Parameter sweep with two variables
echo "Example 2: Parameter sweep with K and M"
parallel -j$(nproc) --bar \
  "../../flowstar-2.1.0/flowstar models/sus_ctrl/sus_test.model -k {1} -m {2}" \
  ::: $(seq 10 5 20) ::: $(seq 5 1 10)

# Example 3: Using a parameter file
echo "Example 3: Using parameter combinations from file"
# Create parameter file
cat > params.txt << EOF
-k 10 -m 5
-k 15 -m 8
-k 20 -m 10
EOF
parallel -j$(nproc) --bar "../../flowstar-2.1.0/flowstar models/sus_ctrl/sus_test.model {}" :::: params.txt

# Example 4: Different models with different parameters
echo "Example 4: Multiple models with parameters"
parallel -j$(nproc) --bar \
  "../../flowstar-2.1.0/flowstar {1} -k {2}" \
  ::: models/sus_ctrl/SUS.model models/sus_ctrl/SUS4.model \
  ::: 10 15 20

# Example 5: Load balanced execution
echo "Example 5: Load balanced execution"
find models -name "*.model" | \
  parallel --load 80% '../../flowstar-2.1.0/flowstar {}'

# Cleanup
rm -f params.txt