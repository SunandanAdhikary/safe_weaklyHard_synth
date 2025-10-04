#!/bin/bash
# Script to run model verification
SYSTEM="suspension_control"
ROOTDIR="/mnt/d/workspace/safe_weaklyHard_synth/"
MODELDIR="$ROOTDIR/models/$SYSTEM/"
MODELFILENAME=""
wsl
cd MODELDIR || exit 1
for file in *.model; do
    MODELFILENAME="$file"
    echo "Running verification for model: $MODELFILENAME"
    ../../flowstar-2.1.0/flowstar < $MODELFILENAME 2>&1 | tee "${MODELFILENAME%.model}.log"
done
cd "$ROOTDIR" || exit 1
echo "Verification completed for all models in $MODELDIR"
# End of script