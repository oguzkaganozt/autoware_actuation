#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091

source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/autoware/setup.bash
"$@" &
ros_pid=$!

# Function to upload the output video to an S3 object
uploadToS3() {
    echo "Uploading output video to S3..."

    if [ -z "$AWS_BATCH_JOB_ARRAY_INDEX" ]; then
        S3_PREFIX="${AWS_BATCH_JOB_ID}"
    else
        S3_PREFIX="${AWS_BATCH_JOB_ID}_${AWS_BATCH_JOB_ARRAY_INDEX}"
    fi
    S3_OBJECT_KEY="${S3_PREFIX}/visu.mp4"
    echo "S3 object key: $S3_OBJECT_KEY"
    # Trim leading/trailing spaces from S3_BUCKET
    S3_BUCKET=$(echo "$S3_BUCKET" | xargs)
    
    FULL_S3_PATH="s3://${S3_BUCKET}/${S3_OBJECT_KEY}"
    echo "Full S3 path: $FULL_S3_PATH"

    # Upload the file to S3
    aws s3 cp ${OUTPUT_FILE} ${FULL_S3_PATH}

    echo "Upload completed."
}

cat_output() {
    echo -e "\e[32mOutput file: $OUTPUT_FILE\e[0m"
    cat $OUTPUT_FILE
}

# Function to handle termination
cleanup() {
    echo "Terminating processes..."
    kill $ros_pid
    wait $ros_pid
    echo "Processes terminated."

    # Call the uploadToS3 function
    if [ -n "$OUTPUT_FILE" ] && [ -f "$OUTPUT_FILE" ]; then
        #uploadToS3 enable once AWS is configured
        cat_output
    else
        echo "Output file not found or not set. Skipping S3 upload."
    fi

    exit 0
}

# Set up trap to catch termination signal
trap cleanup SIGTERM SIGINT

# Wait for any process to exit
wait -n

# No process should exit. If one does, exit the script
exit 1
