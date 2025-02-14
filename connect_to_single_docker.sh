# Function to display an error message and exit
function error_exit {
    echo "$1" 1>&2
    exit 1
}

# Check if DEVELOPER is set
if [ "$DEVELOPER" = "True" ]; then
    echo "DEVELOPER mode detected."

    # Get the list of running container IDs
    running_containers=$(docker ps -q)

    # Check if any containers are running
    if [ -z "$running_containers" ]; then
        error_exit "No running containers found."
    fi

    # Handle single or multiple running containers
    container_count=$(echo "$running_containers" | wc -l)
    if [ "$container_count" -eq 1 ]; then
        # Only one container running, connect to it
        container_id=$(echo "$running_containers")
        echo "Connecting to the running container (ID: $container_id)..."
        docker exec -it "$container_id" /bin/bash || error_exit "Failed to connect to the container."
    elif [ "$container_count" -gt 1 ]; then
        # Multiple containers running, present numbered list
        echo "Multiple containers are running:"
        mapfile -t containers < <(docker ps --format "{{.ID}} {{.Image}} {{.Names}}" | awk 'NF>1')
        for i in "${!containers[@]}"; do
            read -r id image name <<< "${containers[$i]}"
            printf "%2d) %-13s %-20s %s\n" "$((i+1))" "$id"     "$image"     "$name"
        done
        
        echo -n "Enter container number (1-${#containers[@]}): "
        read -r choice
        if [[ ! "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#containers[@]} )); then
            error_exit "Invalid selection"
        fi
        
        container_id=$(echo "${containers[$((choice-1))]}" | awk '{print $1}')
        echo "Connecting to container $container_id..."
        docker exec -it "$container_id" /bin/bash || error_exit "Failed to connect to the container."
    fi
else
    echo "Non-DEVELOPER mode detected."

    # Get the list of running container IDs
    running_containers=$(docker ps -q)

    # Check if any containers are running
    if [ -z "$running_containers" ]; then
        error_exit "No running containers found."
    fi

    # Handle single or multiple running containers
    container_count=$(echo "$running_containers" | wc -l)
    if [ "$container_count" -eq 1 ]; then
        # Only one container running, connect to it
        container_id=$(echo "$running_containers")
        echo "Connecting to the running container and executing startup script (ID: $container_id)..."
        docker exec -it "$container_id" /bin/bash -l -c "/bin/bash /root/ros2_w/src/start_in_docker.sh" || error_exit "Failed to execute startup script in the container."
    elif [ "$container_count" -gt 1 ]; then
        # Multiple containers running, present numbered list
        echo "Multiple containers are running:"
        mapfile -t containers < <(docker ps --format "{{.ID}} {{.Image}} {{.Names}}" | awk 'NF>1')
        for i in "${!containers[@]}"; do
            read -r id image name <<< "${containers[$i]}"
            printf "%2d) %-13s %-20s %s\n" "$((i+1))" "$id"     "$image"     "$name"
        done
        
        echo -n "Enter container number (1-${#containers[@]}): "
        read -r choice
        if [[ ! "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#containers[@]} )); then
            error_exit "Invalid selection"
        fi
        
        container_id=$(echo "${containers[$((choice-1))]}" | awk '{print $1}')
        echo "Connecting to container $container_id and executing startup script..."
        docker exec -it "$container_id" /bin/bash -l -c "/bin/bash /root/ros2_ws/src/start_in_docker.sh" || error_exit "Failed to execute startup script in the container."
    fi
fi
